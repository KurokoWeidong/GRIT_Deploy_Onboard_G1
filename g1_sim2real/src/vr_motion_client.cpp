#include "vr_motion_client.hpp"

#ifdef GRIT_HAVE_ZMQ
#include <zmq.h>
#endif

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace grit_onboard {
#ifdef GRIT_HAVE_ZMQ
namespace {

std::uint64_t monotonic_now_ns()
{
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

void set_int_option(void * socket, int option, int value, const char * name)
{
  if (zmq_setsockopt(socket, option, &value, sizeof(value)) != 0) {
    throw std::runtime_error(std::string("ZeroMQ setsockopt failed for ") + name + ": " + zmq_strerror(errno));
  }
}

void * make_socket(void * context, int type, const std::string & address, int hwm, int hwm_option)
{
  void * socket = zmq_socket(context, type);
  if (socket == nullptr) {
    throw std::runtime_error(std::string("ZeroMQ socket creation failed: ") + zmq_strerror(errno));
  }
  try {
    set_int_option(socket, ZMQ_LINGER, 0, "ZMQ_LINGER");
    set_int_option(socket, hwm_option, hwm, hwm_option == ZMQ_SNDHWM ? "ZMQ_SNDHWM" : "ZMQ_RCVHWM");
    if (zmq_connect(socket, address.c_str()) != 0) {
      throw std::runtime_error("ZeroMQ connect failed for " + address + ": " + zmq_strerror(errno));
    }
  } catch (...) {
    zmq_close(socket);
    throw;
  }
  return socket;
}

std::string receive_part(void * socket, int flags, bool & more)
{
  zmq_msg_t message;
  if (zmq_msg_init(&message) != 0) {
    throw std::runtime_error(std::string("ZeroMQ message init failed: ") + zmq_strerror(errno));
  }
  const int size = zmq_msg_recv(&message, socket, flags);
  if (size < 0) {
    const int saved_errno = errno;
    zmq_msg_close(&message);
    if (saved_errno == EAGAIN) return {};
    throw std::runtime_error(std::string("ZeroMQ receive failed: ") + zmq_strerror(saved_errno));
  }
  std::string data(
      static_cast<const char *>(zmq_msg_data(&message)),
      static_cast<std::size_t>(zmq_msg_size(&message)));
  more = zmq_msg_more(&message) != 0;
  zmq_msg_close(&message);
  return data;
}

bool yaml_bool(const YAML::Node & node, const char * key)
{
  const YAML::Node value = node[key];
  return value && value.as<bool>(false);
}

bool yaml_axis(const YAML::Node & node, const char * key, float & x, float & y)
{
  const YAML::Node value = node[key];
  if (!value || !value.IsSequence() || value.size() < 2) return false;
  try {
    x = value[0].as<float>();
    y = value[1].as<float>();
  } catch (const std::exception &) {
    return false;
  }
  if (!std::isfinite(x) || !std::isfinite(y)) return false;
  x = std::clamp(x, -1.0f, 1.0f);
  y = std::clamp(y, -1.0f, 1.0f);
  return true;
}

}  // namespace

class VrMotionClient::Impl {
 public:
  explicit Impl(VrMotionClientConfig config) : config_(std::move(config))
  {
    context_ = zmq_ctx_new();
    if (context_ == nullptr) {
      throw std::runtime_error(std::string("ZeroMQ context creation failed: ") + zmq_strerror(errno));
    }
    try {
      request_socket_ = make_socket(context_, ZMQ_PUSH, config_.request_address, 100, ZMQ_SNDHWM);
      reply_socket_ = make_socket(context_, ZMQ_PULL, config_.reply_address, 200, ZMQ_RCVHWM);
      control_socket_ = make_socket(context_, ZMQ_PULL, config_.control_address, 200, ZMQ_RCVHWM);
    } catch (...) {
      close();
      throw;
    }
    std::cout << "[VrMotionClient] connected request->" << config_.request_address
              << " reply<-" << config_.reply_address
              << " control<-" << config_.control_address << std::endl;
  }

  ~Impl() { close(); }

  VrOperatorInput poll_operator_input()
  {
    VrOperatorInput input;
    while (true) {
      bool more = false;
      std::string raw;
      try {
        raw = receive_part(control_socket_, ZMQ_DONTWAIT, more);
      } catch (const std::exception & error) {
        std::cerr << "[VrMotionClient] control receive error: " << error.what() << std::endl;
        break;
      }
      if (raw.empty()) break;
      if (more) {
        drain_remaining_parts(control_socket_);
        continue;
      }
      try {
        const YAML::Node parsed = YAML::Load(raw);
        if (parsed && parsed["controller_buttons"] && parsed["controller_buttons"].IsMap()) {
          const YAML::Node buttons = parsed["controller_buttons"];
          const bool x = yaml_bool(buttons, "left_key_one");
          const bool a = yaml_bool(buttons, "right_key_one");
          const bool y = yaml_bool(buttons, "left_key_two");
          const bool b = yaml_bool(buttons, "right_key_two");
          input.activate_or_default = input.activate_or_default || (x && !previous_x_);
          input.start_tracking = input.start_tracking || (a && !previous_a_);
          input.toggle_mode = input.toggle_mode || (b && !previous_b_);
          input.stop_control = input.stop_control || (y && !previous_y_);
          previous_x_ = x;
          previous_a_ = a;
          previous_b_ = b;
          previous_y_ = y;
          float left_x = 0.0f;
          float left_y = 0.0f;
          float right_x = 0.0f;
          float ignored_right_y = 0.0f;
          if (yaml_axis(buttons, "left_axis", left_x, left_y) &&
              yaml_axis(buttons, "right_axis", right_x, ignored_right_y)) {
            latest_left_x_ = left_x;
            latest_left_y_ = left_y;
            latest_right_x_ = right_x;
            latest_sticks_valid_ = true;
          }
          last_control_receive_ns_ = monotonic_now_ns();
        }
      } catch (const std::exception &) {
        std::cerr << "[VrMotionClient] ignored malformed control message" << std::endl;
      }
    }

    constexpr std::uint64_t stick_stale_ns = 250000000ULL;
    const std::uint64_t current_ns = monotonic_now_ns();
    input.sticks_valid = latest_sticks_valid_ && last_control_receive_ns_ > 0 &&
        current_ns - last_control_receive_ns_ <= stick_stale_ns;
    if (input.sticks_valid) {
      input.left_x = latest_left_x_;
      input.left_y = latest_left_y_;
      input.right_x = latest_right_x_;
    }
    return input;
  }

  std::vector<VrPoseFrame> poll_pose_frames()
  {
    std::vector<VrPoseFrame> frames;
    while (true) {
      bool header_more = false;
      std::string header;
      try {
        header = receive_part(reply_socket_, ZMQ_DONTWAIT, header_more);
      } catch (const std::exception & error) {
        std::cerr << "[VrMotionClient] pose receive error: " << error.what() << std::endl;
        break;
      }
      if (header.empty()) break;
      if (!header_more) {
        std::cerr << "[VrMotionClient] ignored pose reply without payload" << std::endl;
        continue;
      }

      bool payload_more = false;
      std::string payload;
      try {
        payload = receive_part(reply_socket_, 0, payload_more);
      } catch (const std::exception & error) {
        std::cerr << "[VrMotionClient] pose payload receive error: " << error.what() << std::endl;
        break;
      }
      if (payload_more) drain_remaining_parts(reply_socket_);

      try {
        const YAML::Node metadata = YAML::Load(header);
        const int count = metadata["num_frames"].as<int>(0);
        const int qpos_size = metadata["qpos_size"].as<int>(0);
        const bool starts_session = metadata["start"].as<bool>(false);
        constexpr int expected_qpos_size = 7 + static_cast<int>(kVrJointCount);
        const std::size_t expected_bytes =
            static_cast<std::size_t>(count) * static_cast<std::size_t>(qpos_size) * sizeof(float);
        if (count <= 0 || qpos_size != expected_qpos_size || payload.size() != expected_bytes) {
          std::cerr << "[VrMotionClient] ignored pose reply with invalid shape or payload length" << std::endl;
          continue;
        }
        const std::uint64_t receive_ns = monotonic_now_ns();
        for (int frame_index = 0; frame_index < count; ++frame_index) {
          std::array<float, expected_qpos_size> qpos{};
          std::memcpy(
              qpos.data(), payload.data() + static_cast<std::size_t>(frame_index) * sizeof(qpos), sizeof(qpos));
          bool finite = true;
          for (float value : qpos) finite = finite && std::isfinite(value);
          const float quat_norm = std::sqrt(
              qpos[3] * qpos[3] + qpos[4] * qpos[4] + qpos[5] * qpos[5] + qpos[6] * qpos[6]);
          if (!finite || quat_norm < 1e-6f) {
            std::cerr << "[VrMotionClient] ignored non-finite/invalid pose frame" << std::endl;
            continue;
          }
          VrPoseFrame frame;
          std::copy_n(qpos.begin(), 3, frame.root_position.begin());
          for (std::size_t i = 0; i < 4; ++i) frame.root_quaternion[i] = qpos[3 + i] / quat_norm;
          std::copy_n(qpos.begin() + 7, kVrJointCount, frame.joint_position.begin());
          frame.starts_session = starts_session && frame_index == 0;
          frame.receive_time_ns = receive_ns;
          frames.push_back(frame);
        }
        if (!frames.empty()) last_pose_receive_ns_ = receive_ns;
      } catch (const std::exception &) {
        std::cerr << "[VrMotionClient] ignored malformed pose reply header" << std::endl;
      }
    }
    return frames;
  }

  bool request_pose(bool starts_session)
  {
    const char * request = starts_session ? "{\"start\":true}" : "{\"start\":false}";
    const int size = static_cast<int>(std::strlen(request));
    const int sent = zmq_send(request_socket_, request, static_cast<std::size_t>(size), ZMQ_DONTWAIT);
    if (sent < 0) {
      if (errno != EAGAIN) {
        std::cerr << "[VrMotionClient] pose request failed: " << zmq_strerror(errno) << std::endl;
      }
      return false;
    }
    return sent == size;
  }

  bool control_received() const { return last_control_receive_ns_ != 0; }
  std::uint64_t last_control_receive_time_ns() const { return last_control_receive_ns_; }
  std::uint64_t last_pose_receive_time_ns() const { return last_pose_receive_ns_; }

 private:
  static void drain_remaining_parts(void * socket)
  {
    bool more = true;
    while (more) {
      try {
        receive_part(socket, 0, more);
      } catch (const std::exception &) {
        break;
      }
    }
  }

  void close()
  {
    if (request_socket_ != nullptr) {
      zmq_close(request_socket_);
      request_socket_ = nullptr;
    }
    if (reply_socket_ != nullptr) {
      zmq_close(reply_socket_);
      reply_socket_ = nullptr;
    }
    if (control_socket_ != nullptr) {
      zmq_close(control_socket_);
      control_socket_ = nullptr;
    }
    if (context_ != nullptr) {
      zmq_ctx_term(context_);
      context_ = nullptr;
    }
  }

  VrMotionClientConfig config_;
  void * context_ = nullptr;
  void * request_socket_ = nullptr;
  void * reply_socket_ = nullptr;
  void * control_socket_ = nullptr;
  bool previous_x_ = false;
  bool previous_a_ = false;
  bool previous_b_ = false;
  bool previous_y_ = false;
  bool latest_sticks_valid_ = false;
  float latest_left_x_ = 0.0f;
  float latest_left_y_ = 0.0f;
  float latest_right_x_ = 0.0f;
  std::uint64_t last_control_receive_ns_ = 0;
  std::uint64_t last_pose_receive_ns_ = 0;
};

#else

class VrMotionClient::Impl {
 public:
  explicit Impl(VrMotionClientConfig)
  {
    throw std::runtime_error(
        "VR mode is unavailable because this build was compiled without ZeroMQ support");
  }

  VrOperatorInput poll_operator_input() { return {}; }
  std::vector<VrPoseFrame> poll_pose_frames() { return {}; }
  bool request_pose(bool) { return false; }
  bool control_received() const { return false; }
  std::uint64_t last_control_receive_time_ns() const { return 0; }
  std::uint64_t last_pose_receive_time_ns() const { return 0; }
};

#endif

VrMotionClient::VrMotionClient(VrMotionClientConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

VrMotionClient::~VrMotionClient() = default;

VrOperatorInput VrMotionClient::poll_operator_input() { return impl_->poll_operator_input(); }

std::vector<VrPoseFrame> VrMotionClient::poll_pose_frames() { return impl_->poll_pose_frames(); }

bool VrMotionClient::request_pose(bool starts_session) { return impl_->request_pose(starts_session); }

bool VrMotionClient::control_received() const { return impl_->control_received(); }

std::uint64_t VrMotionClient::last_control_receive_time_ns() const
{
  return impl_->last_control_receive_time_ns();
}

std::uint64_t VrMotionClient::last_pose_receive_time_ns() const
{
  return impl_->last_pose_receive_time_ns();
}

}  // namespace grit_onboard
