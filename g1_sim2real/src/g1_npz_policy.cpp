#include "npz_reader.hpp"
#include "vr_motion_client.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zlib.h>

#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace grit_onboard {
namespace {

using Clock = std::chrono::steady_clock;
constexpr size_t kDof = 29;
constexpr size_t kFeatureDim = 70;
constexpr size_t kContextFrames = 9;
constexpr size_t kHistoryLength = 10;
constexpr size_t kProprioDim = 99;
constexpr size_t kLocoHistoryLength = 4;
constexpr size_t kLocoFrameDim = 96;
constexpr size_t kLocoObservationDim = kLocoHistoryLength * kLocoFrameDim;
constexpr std::array<size_t, kContextFrames> kContextOffsets{0, 2, 3, 5, 7, 8, 10, 12, 13};
constexpr char kTypeKey[] = "__udp_latest_type__";
constexpr std::array<uint8_t, 4> kMagic{'U', 'L', 'D', 'P'};
constexpr uint8_t kWireVersion = 1;
constexpr size_t kHeaderSize = 32;
constexpr size_t kCrcSize = 4;

std::atomic<bool> g_stop{false};

void handle_signal(int)
{
  g_stop.store(true, std::memory_order_relaxed);
}

uint64_t now_ns()
{
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count());
}

std::string errno_text(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

template <typename T>
T yaml_required(const YAML::Node & node, const std::string & name)
{
  if (!node) {
    throw std::runtime_error("Missing configuration field: " + name);
  }
  return node.as<T>();
}

std::vector<float> yaml_float_vector(const YAML::Node & node, const std::string & name, size_t expected)
{
  if (!node || !node.IsSequence() || node.size() != expected) {
    throw std::runtime_error(name + " must contain " + std::to_string(expected) + " values");
  }
  std::vector<float> out;
  out.reserve(expected);
  for (const auto & value : node) {
    out.push_back(value.as<float>());
  }
  return out;
}

std::vector<std::string> yaml_string_vector(const YAML::Node & node, const std::string & name)
{
  if (!node || !node.IsSequence()) {
    throw std::runtime_error(name + " must be a sequence");
  }
  std::vector<std::string> out;
  out.reserve(node.size());
  for (const auto & value : node) {
    out.push_back(value.as<std::string>());
  }
  return out;
}

std::filesystem::path resolve_path(
    const std::filesystem::path & value, const std::filesystem::path & config_path)
{
  return value.is_absolute() ? value : std::filesystem::weakly_canonical(config_path.parent_path() / value);
}

std::vector<size_t> source_indices(
    const std::vector<std::string> & target, const std::vector<std::string> & source, const std::string & label)
{
  std::unordered_map<std::string, size_t> index;
  for (size_t i = 0; i < source.size(); ++i) {
    if (!index.emplace(source[i], i).second) {
      throw std::runtime_error("Duplicate joint in " + label + ": " + source[i]);
    }
  }
  std::vector<size_t> out;
  out.reserve(target.size());
  for (const auto & name : target) {
    const auto it = index.find(name);
    if (it == index.end()) {
      throw std::runtime_error(label + " is missing joint: " + name);
    }
    out.push_back(it->second);
  }
  return out;
}

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

Vec3 operator+(const Vec3 & a, const Vec3 & b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(const Vec3 & a, const Vec3 & b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(const Vec3 & a, float scale) { return {a.x * scale, a.y * scale, a.z * scale}; }

struct Quat {
  float w = 1.0f;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

Quat normalized(Quat q)
{
  const float norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (!std::isfinite(norm) || norm < 1e-6f) {
    throw std::runtime_error("Invalid zero/non-finite quaternion");
  }
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

Quat conjugate(const Quat & q) { return {q.w, -q.x, -q.y, -q.z}; }

Quat operator*(const Quat & a, const Quat & b)
{
  return normalized({
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w});
}

Vec3 rotate(const Quat & q_in, const Vec3 & value)
{
  const Quat q = normalized(q_in);
  const Vec3 u{q.x, q.y, q.z};
  const float dot_uv = u.x * value.x + u.y * value.y + u.z * value.z;
  const float dot_uu = u.x * u.x + u.y * u.y + u.z * u.z;
  const Vec3 cross{
      u.y * value.z - u.z * value.y,
      u.z * value.x - u.x * value.z,
      u.x * value.y - u.y * value.x};
  return u * (2.0f * dot_uv) + value * (q.w * q.w - dot_uu) + cross * (2.0f * q.w);
}

Quat yaw_quat(const Quat & q_in)
{
  const Quat q = normalized(q_in);
  const float yaw = std::atan2(
      2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
  return {std::cos(0.5f * yaw), 0.0f, 0.0f, std::sin(0.5f * yaw)};
}

Quat replace_yaw(const Quat & reference, const Quat & heading_source)
{
  const Quat tilt = conjugate(yaw_quat(reference)) * normalized(reference);
  Quat aligned = yaw_quat(heading_source) * tilt;
  const float dot =
      aligned.w * reference.w + aligned.x * reference.x +
      aligned.y * reference.y + aligned.z * reference.z;
  if (dot < 0.0f) {
    aligned = {-aligned.w, -aligned.x, -aligned.y, -aligned.z};
  }
  return aligned;
}

Quat slerp(Quat a, Quat b, float t)
{
  a = normalized(a);
  b = normalized(b);
  float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  if (dot < 0.0f) {
    b = {-b.w, -b.x, -b.y, -b.z};
    dot = -dot;
  }
  if (dot > 0.9995f) {
    return normalized({
        a.w + t * (b.w - a.w), a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)});
  }
  dot = std::clamp(dot, -1.0f, 1.0f);
  const float theta = std::acos(dot);
  const float denom = std::sin(theta);
  const float wa = std::sin((1.0f - t) * theta) / denom;
  const float wb = std::sin(t * theta) / denom;
  return normalized({
      wa * a.w + wb * b.w, wa * a.x + wb * b.x,
      wa * a.y + wb * b.y, wa * a.z + wb * b.z});
}

std::array<float, 9> rotation_matrix(const Quat & q_in)
{
  const Quat q = normalized(q_in);
  const float ww = q.w * q.w;
  const float xx = q.x * q.x;
  const float yy = q.y * q.y;
  const float zz = q.z * q.z;
  return {
      ww + xx - yy - zz, 2.0f * (q.x * q.y - q.w * q.z), 2.0f * (q.x * q.z + q.w * q.y),
      2.0f * (q.x * q.y + q.w * q.z), ww - xx + yy - zz, 2.0f * (q.y * q.z - q.w * q.x),
      2.0f * (q.x * q.z - q.w * q.y), 2.0f * (q.y * q.z + q.w * q.x), ww - xx - yy + zz};
}

std::array<float, 6> rot6d(const Quat & q)
{
  const auto m = rotation_matrix(q);
  return {m[0], m[1], m[3], m[4], m[6], m[7]};
}

Vec3 rotation_vector(const Quat & q_in)
{
  Quat q = normalized(q_in);
  if (q.w < 0.0f) {
    q = {-q.w, -q.x, -q.y, -q.z};
  }
  const float sin_half = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  if (sin_half < 1e-7f) {
    return {2.0f * q.x, 2.0f * q.y, 2.0f * q.z};
  }
  const float scale = 2.0f * std::atan2(sin_half, q.w) / sin_half;
  return {q.x * scale, q.y * scale, q.z * scale};
}

struct Motion {
  size_t frames = 0;
  float fps = 50.0f;
  std::vector<float> joint_pos;
  std::vector<float> joint_vel;
  std::vector<Vec3> root_pos;
  std::vector<Quat> root_quat;
  std::vector<Vec3> root_lin_vel_b;
  std::vector<Vec3> root_ang_vel_b;
  bool exported_velocities = true;
};

void require_shape(const NpyArray & array, const std::vector<size_t> & expected, const std::string & name)
{
  if (array.shape != expected) {
    std::ostringstream message;
    message << "NPZ " << name << " has shape [";
    for (size_t i = 0; i < array.shape.size(); ++i) {
      message << (i == 0 ? "" : ",") << array.shape[i];
    }
    message << "] but expected [";
    for (size_t i = 0; i < expected.size(); ++i) {
      message << (i == 0 ? "" : ",") << expected[i];
    }
    message << "]";
    throw std::runtime_error(message.str());
  }
}

Motion load_motion(const std::string & path)
{
  NpzArchive archive(path);
  const auto fps_values = archive.at("fps").as_float();
  if (fps_values.size() != 1 || !std::isfinite(fps_values[0]) || std::abs(fps_values[0] - 50.0f) > 1e-3f) {
    throw std::runtime_error("NPZ fps must be exactly 50 Hz");
  }
  const NpyArray & joint_pos_array = archive.at("joint_pos");
  if (joint_pos_array.shape.size() != 2 || joint_pos_array.shape[0] == 0 || joint_pos_array.shape[1] != kDof) {
    throw std::runtime_error("NPZ joint_pos must have shape [T, 29] with T > 0");
  }
  const size_t frames = joint_pos_array.shape[0];
  require_shape(archive.at("joint_vel"), {frames, kDof}, "joint_vel");
  const NpyArray & body_pos_array = archive.at("body_pos_w");
  if (body_pos_array.shape.size() != 3 || body_pos_array.shape[0] != frames ||
      body_pos_array.shape[1] < 1 || body_pos_array.shape[2] != 3) {
    throw std::runtime_error("NPZ body_pos_w must have shape [T, B, 3] with B > 0");
  }
  const size_t bodies = body_pos_array.shape[1];
  require_shape(archive.at("body_quat_w"), {frames, bodies, 4}, "body_quat_w");
  require_shape(archive.at("body_lin_vel_w"), {frames, bodies, 3}, "body_lin_vel_w");
  require_shape(archive.at("body_ang_vel_w"), {frames, bodies, 3}, "body_ang_vel_w");

  Motion motion;
  motion.frames = frames;
  motion.fps = fps_values[0];
  motion.joint_pos = joint_pos_array.as_float();
  motion.joint_vel = archive.at("joint_vel").as_float();
  const auto body_pos = body_pos_array.as_float();
  const auto body_quat = archive.at("body_quat_w").as_float();
  const auto body_lin_vel = archive.at("body_lin_vel_w").as_float();
  const auto body_ang_vel = archive.at("body_ang_vel_w").as_float();
  motion.root_pos.resize(frames);
  motion.root_quat.resize(frames);
  motion.root_lin_vel_b.resize(frames);
  motion.root_ang_vel_b.resize(frames);
  for (size_t t = 0; t < frames; ++t) {
    const size_t pos = t * bodies * 3;
    const size_t quat = t * bodies * 4;
    motion.root_pos[t] = {body_pos[pos], body_pos[pos + 1], body_pos[pos + 2]};
    motion.root_quat[t] = normalized({
        body_quat[quat], body_quat[quat + 1], body_quat[quat + 2], body_quat[quat + 3]});
    const Vec3 lin_w{body_lin_vel[pos], body_lin_vel[pos + 1], body_lin_vel[pos + 2]};
    const Vec3 ang_w{body_ang_vel[pos], body_ang_vel[pos + 1], body_ang_vel[pos + 2]};
    motion.root_lin_vel_b[t] = rotate(conjugate(motion.root_quat[t]), lin_w);
    motion.root_ang_vel_b[t] = rotate(conjugate(motion.root_quat[t]), ang_w);
  }
  const auto finite = [](float value) { return std::isfinite(value); };
  if (!std::all_of(motion.joint_pos.begin(), motion.joint_pos.end(), finite) ||
      !std::all_of(motion.joint_vel.begin(), motion.joint_vel.end(), finite)) {
    throw std::runtime_error("NPZ joint arrays contain non-finite values");
  }
  std::cout << "[G1NpzPolicy] validated motion=" << path << " frames=" << frames
            << " fps=" << motion.fps << std::endl;
  return motion;
}

std::vector<std::filesystem::path> discover_motion_paths(const std::filesystem::path & input)
{
  std::vector<std::filesystem::path> paths;
  if (std::filesystem::is_regular_file(input)) {
    if (input.extension() != ".npz") {
      throw std::runtime_error("Motion file must have the .npz extension: " + input.string());
    }
    paths.push_back(input);
  } else if (std::filesystem::is_directory(input)) {
    for (const auto & entry : std::filesystem::directory_iterator(input)) {
      if (entry.is_regular_file() && entry.path().extension() == ".npz") {
        paths.push_back(std::filesystem::weakly_canonical(entry.path()));
      }
    }
    std::sort(paths.begin(), paths.end(), [](const auto & a, const auto & b) {
      return a.filename().string() < b.filename().string();
    });
    if (paths.empty()) {
      throw std::runtime_error("Motion directory contains no .npz files: " + input.string());
    }
  } else {
    throw std::runtime_error("Motion path is neither a file nor a directory: " + input.string());
  }
  std::cout << "[G1DualPolicy] motion playlist contains " << paths.size()
            << " item(s); first=" << paths.front().filename().string() << std::endl;
  return paths;
}

Motion default_motion(const YAML::Node & tracking)
{
  const YAML::Node clips = tracking["motion_clips"];
  if (!clips || !clips.IsSequence()) {
    throw std::runtime_error("tracking.motion_clips is required");
  }
  for (const auto & clip : clips) {
    if (clip["name"] && clip["name"].as<std::string>() == "default") {
      Motion motion;
      motion.frames = 1;
      motion.joint_pos = yaml_float_vector(clip["joint_pos"], "default.joint_pos", kDof);
      motion.joint_vel.assign(kDof, 0.0f);
      const auto pos = yaml_float_vector(clip["root_pos"], "default.root_pos", 3);
      const auto quat = yaml_float_vector(clip["root_quat"], "default.root_quat", 4);
      motion.root_pos.push_back({pos[0], pos[1], pos[2]});
      motion.root_quat.push_back(normalized({quat[0], quat[1], quat[2], quat[3]}));
      motion.root_lin_vel_b.push_back({});
      motion.root_ang_vel_b.push_back({});
      motion.exported_velocities = false;
      return motion;
    }
  }
  throw std::runtime_error("tracking.motion_clips must include default");
}

struct PackedArray {
  size_t offset = 0;
  size_t bytes = 0;
  size_t count = 0;
};

void append_u16_be(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>(value >> 8));
  out.push_back(static_cast<uint8_t>(value));
}

void append_u32_be(std::vector<uint8_t> & out, uint32_t value)
{
  for (int shift = 24; shift >= 0; shift -= 8) out.push_back(static_cast<uint8_t>(value >> shift));
}

void append_u64_be(std::vector<uint8_t> & out, uint64_t value)
{
  for (int shift = 56; shift >= 0; shift -= 8) out.push_back(static_cast<uint8_t>(value >> shift));
}

uint32_t read_u32_be(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
         (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
}

uint64_t read_u64_be(const uint8_t * p)
{
  uint64_t value = 0;
  for (size_t i = 0; i < 8; ++i) value = (value << 8) | p[i];
  return value;
}

PackedArray append_array(std::vector<uint8_t> & payload, const std::vector<float> & values)
{
  PackedArray ref{payload.size(), values.size() * sizeof(float), values.size()};
  payload.resize(payload.size() + ref.bytes);
  std::memcpy(payload.data() + ref.offset, values.data(), ref.bytes);
  return ref;
}

std::string array_meta(const PackedArray & ref)
{
  std::ostringstream out;
  out << "{\"" << kTypeKey << "\":\"ndarray\",\"dtype\":\"<f4\",\"shape\":["
      << ref.count << "],\"offset\":" << ref.offset << ",\"nbytes\":" << ref.bytes << "}";
  return out.str();
}

std::string json_string(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20) {
          out << "\\u00" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<int>(ch) << std::dec << std::setfill(' ');
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  out << '"';
  return out.str();
}

std::string motion_voice_label(const std::filesystem::path & path)
{
  return path.stem().string();
}

std::vector<uint8_t> encode_packet(
    const std::string & meta, const std::vector<uint8_t> & payload, uint64_t seq)
{
  std::vector<uint8_t> packet;
  packet.reserve(kHeaderSize + kCrcSize + meta.size() + payload.size());
  packet.insert(packet.end(), kMagic.begin(), kMagic.end());
  packet.push_back(kWireVersion);
  packet.push_back(0);
  append_u16_be(packet, 0);
  append_u64_be(packet, seq);
  append_u64_be(packet, now_ns());
  append_u32_be(packet, static_cast<uint32_t>(meta.size()));
  append_u32_be(packet, static_cast<uint32_t>(payload.size()));
  uLong crc = crc32(0, packet.data(), static_cast<uInt>(packet.size()));
  crc = crc32(crc, reinterpret_cast<const Bytef *>(meta.data()), static_cast<uInt>(meta.size()));
  crc = crc32(crc, payload.data(), static_cast<uInt>(payload.size()));
  append_u32_be(packet, static_cast<uint32_t>(crc));
  packet.insert(packet.end(), meta.begin(), meta.end());
  packet.insert(packet.end(), payload.begin(), payload.end());
  return packet;
}

std::vector<float> read_array(
    const YAML::Node & root, const std::vector<uint8_t> & payload, const std::string & key, size_t expected)
{
  const YAML::Node meta = root[key];
  if (!meta || meta[kTypeKey].as<std::string>("") != "ndarray" || meta["dtype"].as<std::string>("") != "<f4") {
    throw std::runtime_error("State field is not float32 ndarray: " + key);
  }
  const size_t offset = meta["offset"].as<size_t>();
  const size_t bytes = meta["nbytes"].as<size_t>();
  if (bytes != expected * sizeof(float) || offset > payload.size() || bytes > payload.size() - offset) {
    throw std::runtime_error("State field has invalid size: " + key);
  }
  std::vector<float> out(expected);
  std::memcpy(out.data(), payload.data() + offset, bytes);
  return out;
}

struct RobotState {
  uint64_t seq = 0;
  uint64_t receive_time_ns = 0;
  std::vector<float> q;
  std::vector<float> dq;
  Quat quat;
  Vec3 gyro;
  bool start = false;
  bool a = false;
  bool b = false;
  bool x = false;
  bool left = false;
  bool right = false;
  float lx = 0.0f;
  float ly = 0.0f;
  float rx = 0.0f;
  float ry = 0.0f;
};

class RobotTransport {
 public:
  RobotTransport(const YAML::Node & udp)
  {
    const std::string bind_host = yaml_required<std::string>(udp["state_bind_host"], "udp.state_bind_host");
    const int state_port = yaml_required<int>(udp["state_port"], "udp.state_port");
    const std::string command_host = yaml_required<std::string>(udp["cmd_host"], "udp.cmd_host");
    const int command_port = yaml_required<int>(udp["cmd_port"], "udp.cmd_port");
    recv_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    send_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (recv_fd_ < 0 || send_fd_ < 0) throw std::runtime_error(errno_text("socket failed"));
    const int reuse = 1;
    ::setsockopt(recv_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in bind{};
    bind.sin_family = AF_INET;
    bind.sin_port = htons(static_cast<uint16_t>(state_port));
    if (bind_host.empty() || bind_host == "0.0.0.0") bind.sin_addr.s_addr = htonl(INADDR_ANY);
    else if (::inet_pton(AF_INET, bind_host.c_str(), &bind.sin_addr) != 1) throw std::runtime_error("Bad bind host");
    if (::bind(recv_fd_, reinterpret_cast<sockaddr *>(&bind), sizeof(bind)) < 0) {
      throw std::runtime_error(errno_text("state UDP bind failed"));
    }
    command_target_.sin_family = AF_INET;
    command_target_.sin_port = htons(static_cast<uint16_t>(command_port));
    if (::inet_pton(AF_INET, command_host.c_str(), &command_target_.sin_addr) != 1) {
      throw std::runtime_error("Bad command host: " + command_host);
    }
    const int flags = ::fcntl(recv_fd_, F_GETFL, 0);
    ::fcntl(recv_fd_, F_SETFL, flags | O_NONBLOCK);
  }

  ~RobotTransport()
  {
    if (recv_fd_ >= 0) ::close(recv_fd_);
    if (send_fd_ >= 0) ::close(send_fd_);
  }

  bool receive(RobotState & state, int timeout_ms)
  {
    pollfd descriptor{recv_fd_, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1, timeout_ms);
    if (ready <= 0 || !(descriptor.revents & POLLIN)) return false;
    std::vector<uint8_t> packet;
    std::array<uint8_t, 65535> buffer{};
    while (true) {
      const ssize_t count = ::recvfrom(recv_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
      if (count < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        throw std::runtime_error(errno_text("state recvfrom failed"));
      }
      packet.assign(buffer.begin(), buffer.begin() + count);
      state.receive_time_ns = now_ns();
    }
    if (packet.size() < kHeaderSize + kCrcSize || !std::equal(kMagic.begin(), kMagic.end(), packet.begin())) {
      throw std::runtime_error("Invalid state UDP packet");
    }
    const size_t meta_size = read_u32_be(packet.data() + 24);
    const size_t payload_size = read_u32_be(packet.data() + 28);
    if (packet.size() != kHeaderSize + kCrcSize + meta_size + payload_size) {
      throw std::runtime_error("State UDP packet length mismatch");
    }
    const uint32_t received_crc = read_u32_be(packet.data() + kHeaderSize);
    uLong crc = crc32(0, packet.data(), kHeaderSize);
    crc = crc32(crc, packet.data() + kHeaderSize + kCrcSize, static_cast<uInt>(meta_size + payload_size));
    if (static_cast<uint32_t>(crc) != received_crc) throw std::runtime_error("State UDP CRC mismatch");
    const char * meta_start = reinterpret_cast<const char *>(packet.data() + kHeaderSize + kCrcSize);
    const YAML::Node meta = YAML::Load(std::string(meta_start, meta_start + meta_size));
    std::vector<uint8_t> payload(
        packet.begin() + static_cast<std::ptrdiff_t>(kHeaderSize + kCrcSize + meta_size), packet.end());
    state.seq = read_u64_be(packet.data() + 8);
    state.q = read_array(meta, payload, "q", kDof);
    state.dq = read_array(meta, payload, "dq", kDof);
    const auto quat = read_array(meta, payload, "quat_wxyz", 4);
    const auto gyro = read_array(meta, payload, "gyro", 3);
    state.quat = normalized({quat[0], quat[1], quat[2], quat[3]});
    state.gyro = {gyro[0], gyro[1], gyro[2]};
    state.start = meta["buttons"]["start"].as<bool>(false);
    state.a = meta["buttons"]["A"].as<bool>(false);
    state.b = meta["buttons"]["B"].as<bool>(false);
    state.x = meta["buttons"]["stop"].as<bool>(false);
    state.left = meta["buttons"]["left"].as<bool>(false);
    state.right = meta["buttons"]["right"].as<bool>(false);
    state.lx = meta["sticks"]["lx"].as<float>(0.0f);
    state.ly = meta["sticks"]["ly"].as<float>(0.0f);
    state.rx = meta["sticks"]["rx"].as<float>(0.0f);
    state.ry = meta["sticks"]["ry"].as<float>(0.0f);
    if (!std::isfinite(state.lx) || !std::isfinite(state.ly) ||
        !std::isfinite(state.rx) || !std::isfinite(state.ry)) {
      throw std::runtime_error("Remote stick state contains a non-finite value");
    }
    return true;
  }

  void send(
      const std::vector<float> & q, const std::vector<float> & qd,
      const std::vector<float> & kp, const std::vector<float> & kd,
      int enable, uint64_t state_receive_time_ns)
  {
    std::vector<uint8_t> payload;
    const PackedArray q_ref = append_array(payload, q);
    const PackedArray qd_ref = append_array(payload, qd);
    const PackedArray kp_ref = append_array(payload, kp);
    const PackedArray kd_ref = append_array(payload, kd);
    std::ostringstream meta;
    meta << "{\"q_des\":" << array_meta(q_ref) << ",\"qd_des\":" << array_meta(qd_ref)
         << ",\"kp\":" << array_meta(kp_ref) << ",\"kd\":" << array_meta(kd_ref)
         << ",\"enable\":" << enable << ",\"state_receive_time_ns\":" << state_receive_time_ns
         << ",\"announcement_id\":" << announcement_id_
         << ",\"announcement\":" << json_string(announcement_)
         << ",\"mode_indicator_id\":" << mode_indicator_id_
         << ",\"mode_indicator\":" << json_string(mode_indicator_) << "}";
    const auto packet = encode_packet(meta.str(), payload, sequence_++);
    const ssize_t count = ::sendto(
        send_fd_, packet.data(), packet.size(), 0,
        reinterpret_cast<const sockaddr *>(&command_target_), sizeof(command_target_));
    if (count < 0 || static_cast<size_t>(count) != packet.size()) {
      throw std::runtime_error(errno_text("command sendto failed"));
    }
  }

  void announce(const std::string & message)
  {
    announcement_ = message;
    ++announcement_id_;
    std::cout << "[G1DualPolicy] queued voice announcement " << announcement_id_
              << ": " << message << std::endl;
  }

  void announce_mode(const std::string & message, const std::string & mode_indicator)
  {
    announce(message);
    mode_indicator_ = mode_indicator;
    ++mode_indicator_id_;
    std::cout << "[G1DualPolicy] queued mode indicator " << mode_indicator_id_
              << ": " << mode_indicator << std::endl;
  }

 private:
  int recv_fd_ = -1;
  int send_fd_ = -1;
  sockaddr_in command_target_{};
  uint64_t sequence_ = 0;
  uint64_t announcement_id_ = 0;
  std::string announcement_;
  uint64_t mode_indicator_id_ = 0;
  std::string mode_indicator_;
};

struct ReferenceSegment {
  size_t frames = 0;
  size_t index = 0;
  std::vector<float> joints;
  std::vector<Vec3> positions;
  std::vector<Quat> quaternions;
  std::vector<float> features;
};

std::vector<float> finite_difference_features(
    const std::vector<float> & joints, const std::vector<Vec3> & positions,
    const std::vector<Quat> & quaternions, float fps)
{
  const size_t frames = positions.size();
  std::vector<float> features(frames * kFeatureDim, 0.0f);
  auto sample_index = [frames](size_t t, int delta) {
    const int value = static_cast<int>(t) + delta;
    return static_cast<size_t>(std::clamp(value, 0, static_cast<int>(frames) - 1));
  };
  for (size_t t = 0; t < frames; ++t) {
    float * out = features.data() + t * kFeatureDim;
    std::copy_n(joints.data() + t * kDof, kDof, out);
    const size_t before = sample_index(t, -1);
    const size_t after = sample_index(t, 1);
    const float derivative_scale = (t == 0 || t + 1 == frames) ? fps : (0.5f * fps);
    for (size_t j = 0; j < kDof; ++j) {
      out[kDof + j] = (joints[after * kDof + j] - joints[before * kDof + j]) * derivative_scale;
    }
    const auto orientation = rot6d(quaternions[t]);
    std::copy(orientation.begin(), orientation.end(), out + 58);
    const Vec3 velocity_w = (positions[after] - positions[before]) * derivative_scale;
    const Vec3 velocity_b = rotate(conjugate(quaternions[t]), velocity_w);
    out[64] = velocity_b.x;
    out[65] = velocity_b.y;
    out[66] = velocity_b.z;
    Vec3 angular;
    if (frames > 1) {
      if (t == 0) {
        angular = rotation_vector(conjugate(quaternions[t]) * quaternions[after]) * fps;
      } else if (t + 1 == frames) {
        angular = rotation_vector(conjugate(quaternions[before]) * quaternions[t]) * fps;
      } else {
        const Vec3 forward = rotation_vector(conjugate(quaternions[t]) * quaternions[after]);
        const Vec3 backward = rotation_vector(conjugate(quaternions[t]) * quaternions[before]);
        angular = (forward - backward) * (0.5f * fps);
      }
    }
    out[67] = angular.x;
    out[68] = angular.y;
    out[69] = angular.z;
  }
  return features;
}

ReferenceSegment build_segment(
    const Motion & source, const std::vector<float> & anchor_joint,
    const Vec3 & anchor_pos, const Quat & anchor_quat, size_t transition_steps)
{
  const Quat yaw_delta = yaw_quat(anchor_quat) * conjugate(yaw_quat(source.root_quat.front()));
  std::vector<Vec3> aligned_pos(source.frames);
  std::vector<Quat> aligned_quat(source.frames);
  for (size_t t = 0; t < source.frames; ++t) {
    aligned_pos[t] = rotate(yaw_delta, source.root_pos[t] - source.root_pos.front()) + anchor_pos;
    aligned_pos[t].z = source.root_pos[t].z;
    aligned_quat[t] = yaw_delta * source.root_quat[t];
  }
  ReferenceSegment segment;
  segment.frames = transition_steps + source.frames;
  segment.joints.resize(segment.frames * kDof);
  segment.positions.resize(segment.frames);
  segment.quaternions.resize(segment.frames);
  for (size_t t = 0; t < transition_steps; ++t) {
    const float alpha = static_cast<float>(t + 1) / static_cast<float>(transition_steps + 1);
    for (size_t j = 0; j < kDof; ++j) {
      segment.joints[t * kDof + j] =
          anchor_joint[j] * (1.0f - alpha) + source.joint_pos[j] * alpha;
    }
    segment.positions[t] = anchor_pos * (1.0f - alpha) + aligned_pos.front() * alpha;
    segment.quaternions[t] = slerp(anchor_quat, aligned_quat.front(), alpha);
  }
  std::copy(source.joint_pos.begin(), source.joint_pos.end(), segment.joints.begin() + transition_steps * kDof);
  std::copy(aligned_pos.begin(), aligned_pos.end(), segment.positions.begin() + transition_steps);
  std::copy(aligned_quat.begin(), aligned_quat.end(), segment.quaternions.begin() + transition_steps);

  if (!source.exported_velocities) {
    segment.features = finite_difference_features(
        segment.joints, segment.positions, segment.quaternions, source.fps);
    return segment;
  }
  segment.features.resize(segment.frames * kFeatureDim);
  if (transition_steps > 0) {
    std::vector<float> prefix_joints(segment.joints.begin(), segment.joints.begin() + transition_steps * kDof);
    std::vector<Vec3> prefix_pos(segment.positions.begin(), segment.positions.begin() + transition_steps);
    std::vector<Quat> prefix_quat(segment.quaternions.begin(), segment.quaternions.begin() + transition_steps);
    const auto prefix_features = finite_difference_features(prefix_joints, prefix_pos, prefix_quat, source.fps);
    std::copy(prefix_features.begin(), prefix_features.end(), segment.features.begin());
  }
  for (size_t t = 0; t < source.frames; ++t) {
    float * out = segment.features.data() + (transition_steps + t) * kFeatureDim;
    std::copy_n(source.joint_pos.data() + t * kDof, kDof, out);
    std::copy_n(source.joint_vel.data() + t * kDof, kDof, out + kDof);
    const auto orientation = rot6d(aligned_quat[t]);
    std::copy(orientation.begin(), orientation.end(), out + 58);
    out[64] = source.root_lin_vel_b[t].x;
    out[65] = source.root_lin_vel_b[t].y;
    out[66] = source.root_lin_vel_b[t].z;
    out[67] = source.root_ang_vel_b[t].x;
    out[68] = source.root_ang_vel_b[t].y;
    out[69] = source.root_ang_vel_b[t].z;
  }
  return segment;
}

class InferenceModel {
 public:
  InferenceModel(const std::string & path, int threads)
      : env_(ORT_LOGGING_LEVEL_WARNING, "grit_onboard"), options_{}, session_{nullptr}
  {
    options_.SetIntraOpNumThreads(std::max(1, threads));
    options_.SetInterOpNumThreads(1);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, path.c_str(), options_);
    Ort::AllocatorWithDefaultOptions allocator;
    if (session_.GetInputCount() != 2 || session_.GetOutputCount() != 1) {
      throw std::runtime_error("GRIT ONNX must have exactly two inputs and one output");
    }
    const auto input0 = session_.GetInputNameAllocated(0, allocator);
    const auto input1 = session_.GetInputNameAllocated(1, allocator);
    const auto output = session_.GetOutputNameAllocated(0, allocator);
    if (std::string(input0.get()) != "reference_context" ||
        std::string(input1.get()) != "proprio_history" || std::string(output.get()) != "actions") {
      throw std::runtime_error("GRIT ONNX input/output names do not match the deployment contract");
    }
    std::vector<float> ref(kContextFrames * kFeatureDim, 0.0f);
    std::vector<float> proprio(kHistoryLength * kProprioDim, 0.0f);
    for (int i = 0; i < 20; ++i) infer(ref, proprio);
    std::vector<double> samples;
    samples.reserve(100);
    for (int i = 0; i < 100; ++i) {
      const auto begin = Clock::now();
      infer(ref, proprio);
      samples.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin).count());
    }
    std::sort(samples.begin(), samples.end());
    const double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    std::cout << std::fixed << std::setprecision(3)
              << "[G1NpzPolicy] ONNX benchmark model=" << path << " threads=" << threads
              << " mean=" << mean << " ms p50=" << samples[50] << " ms p95=" << samples[95]
              << " ms p99=" << samples[99] << " ms" << std::endl;
  }

  std::vector<float> infer(std::vector<float> & reference, std::vector<float> & proprio)
  {
    const std::array<int64_t, 3> ref_shape{1, static_cast<int64_t>(kContextFrames), static_cast<int64_t>(kFeatureDim)};
    const std::array<int64_t, 2> proprio_shape{1, static_cast<int64_t>(proprio.size())};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(memory, reference.data(), reference.size(), ref_shape.data(), ref_shape.size()),
        Ort::Value::CreateTensor<float>(memory, proprio.data(), proprio.size(), proprio_shape.data(), proprio_shape.size())};
    const char * input_names[] = {"reference_context", "proprio_history"};
    const char * output_names[] = {"actions"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names, inputs.data(), inputs.size(), output_names, 1);
    const auto info = outputs[0].GetTensorTypeAndShapeInfo();
    if (info.GetElementCount() != kDof) throw std::runtime_error("ONNX actions output must contain 29 values");
    const float * actions = outputs[0].GetTensorData<float>();
    return std::vector<float>(actions, actions + kDof);
  }

 private:
  Ort::Env env_;
  Ort::SessionOptions options_;
  Ort::Session session_;
};

class LocomotionInferenceModel {
 public:
  LocomotionInferenceModel(const std::string & path, int threads)
      : env_(ORT_LOGGING_LEVEL_WARNING, "g1_locomotion"), options_{}, session_{nullptr}
  {
    options_.SetIntraOpNumThreads(std::max(1, threads));
    options_.SetInterOpNumThreads(1);
    options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, path.c_str(), options_);
    Ort::AllocatorWithDefaultOptions allocator;
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1) {
      throw std::runtime_error("Locomotion ONNX must have exactly one input and one output");
    }
    const auto input = session_.GetInputNameAllocated(0, allocator);
    const auto output = session_.GetOutputNameAllocated(0, allocator);
    if (std::string(input.get()) != "obs" || std::string(output.get()) != "actions") {
      throw std::runtime_error("Locomotion ONNX input/output names must be obs/actions");
    }
    const auto input_shape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto output_shape = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    if (input_shape.size() != 2 || input_shape[1] != static_cast<int64_t>(kLocoObservationDim) ||
        output_shape.size() != 2 || output_shape[1] != static_cast<int64_t>(kDof)) {
      throw std::runtime_error("Locomotion ONNX shapes must be obs [N,384] and actions [N,29]");
    }

    std::vector<float> observation(kLocoObservationDim, 0.0f);
    for (int i = 0; i < 20; ++i) infer(observation);
    std::vector<double> samples;
    samples.reserve(100);
    for (int i = 0; i < 100; ++i) {
      const auto begin = Clock::now();
      infer(observation);
      samples.push_back(std::chrono::duration<double, std::milli>(Clock::now() - begin).count());
    }
    std::sort(samples.begin(), samples.end());
    const double mean = std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    std::cout << std::fixed << std::setprecision(3)
              << "[G1DualPolicy] locomotion ONNX benchmark model=" << path << " threads=" << threads
              << " mean=" << mean << " ms p50=" << samples[50] << " ms p95=" << samples[95]
              << " ms p99=" << samples[99] << " ms" << std::endl;
  }

  std::vector<float> infer(std::vector<float> & observation)
  {
    const std::array<int64_t, 2> shape{1, static_cast<int64_t>(observation.size())};
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto input = Ort::Value::CreateTensor<float>(
        memory, observation.data(), observation.size(), shape.data(), shape.size());
    const char * input_names[] = {"obs"};
    const char * output_names[] = {"actions"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
    const auto info = outputs[0].GetTensorTypeAndShapeInfo();
    if (info.GetElementCount() != kDof) {
      throw std::runtime_error("Locomotion ONNX actions output must contain 29 values");
    }
    const float * actions = outputs[0].GetTensorData<float>();
    return std::vector<float>(actions, actions + kDof);
  }

 private:
  Ort::Env env_;
  Ort::SessionOptions options_;
  Ort::Session session_;
};

class LocomotionPolicy {
 public:
  struct StickInput {
    float left_x = 0.0f;
    float left_y = 0.0f;
    float right_x = 0.0f;
  };

  LocomotionPolicy(
      const YAML::Node & controller, const YAML::Node & tracking,
      const std::string & model_path, int threads)
      : default_motion_(default_motion(tracking)), model_(model_path, threads)
  {
    controller_names_ = yaml_string_vector(controller["policy_joint_names"], "controller.policy_joint_names");
    reference_names_ = yaml_string_vector(tracking["reference_joint_names"], "tracking.reference_joint_names");
    if (controller_names_.size() != kDof || reference_names_.size() != kDof) {
      throw std::runtime_error("Controller and reference joint lists must each contain 29 names");
    }
    ref_from_controller_ = source_indices(reference_names_, controller_names_, "controller-to-reference map");
    controller_from_ref_ = source_indices(controller_names_, reference_names_, "reference-to-controller map");
    const std::vector<float> expected_default{
        -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f,
        -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.2f, 0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f,
        0.2f, -0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < kDof; ++i) {
      if (std::abs(default_motion_.joint_pos[i] - expected_default[i]) > 1e-5f) {
        throw std::runtime_error("GRIT default pose does not match the locomotion training contract");
      }
    }
    const std::vector<float> armature{
        0.025101925f, 0.025101925f, 0.010177520f, 0.025101925f, 0.003609725f, 0.003609725f,
        0.025101925f, 0.025101925f, 0.010177520f, 0.025101925f, 0.003609725f, 0.003609725f,
        0.010177520f, 0.003609725f, 0.003609725f,
        0.003609725f, 0.003609725f, 0.003609725f, 0.003609725f, 0.003609725f,
        0.0021812f, 0.0021812f,
        0.003609725f, 0.003609725f, 0.003609725f, 0.003609725f, 0.003609725f,
        0.0021812f, 0.0021812f};
    const std::vector<float> gain_multiplier{
        1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 2, 2, 1, 2, 2,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    const std::vector<float> torque_limit{
        139, 139, 88, 139, 50, 50, 139, 139, 88, 139, 50, 50, 88, 50, 50,
        25, 25, 25, 25, 25, 10, 10, 25, 25, 25, 25, 25, 10, 10};
    constexpr float natural_frequency = 10.0f * 2.0f * 3.1415926535f;
    std::vector<float> kp_ref(kDof);
    std::vector<float> kd_ref(kDof);
    action_scale_.resize(kDof);
    for (size_t i = 0; i < kDof; ++i) {
      kp_ref[i] = gain_multiplier[i] * armature[i] * natural_frequency * natural_frequency;
      kd_ref[i] = gain_multiplier[i] * 4.0f * armature[i] * natural_frequency;
      action_scale_[i] = 0.25f * torque_limit[i] / kp_ref[i];
    }
    kps_ = map_from_reference(kp_ref);
    kds_ = map_from_reference(kd_ref);
    last_action_.assign(kDof, 0.0f);
    history_.reserve(kLocoObservationDim);
  }

  void reset(const RobotState & state)
  {
    last_action_.assign(kDof, 0.0f);
    previous_command_ = {0.0f, 0.0f, 0.0f};
    history_.clear();
    const auto frame = make_frame(state, {0.0f, 0.0f, 0.0f});
    for (size_t i = 0; i < kLocoHistoryLength; ++i) {
      history_.insert(history_.end(), frame.begin(), frame.end());
    }
  }

  bool sticks_neutral(
      const RobotState & state, const std::optional<StickInput> & pico_sticks) const
  {
    const bool remote_neutral =
        std::abs(state.lx) <= kDeadZone && std::abs(state.ly) <= kDeadZone &&
        std::abs(state.rx) <= kDeadZone;
    return remote_neutral && (!pico_sticks.has_value() || sticks_neutral(*pico_sticks));
  }

  const std::vector<float> & kps() const { return kps_; }
  const std::vector<float> & kds() const { return kds_; }

  std::vector<float> step(
      const RobotState & state, bool force_zero_command,
      const std::optional<StickInput> & pico_sticks, double & inference_ms)
  {
    const bool pico_active = pico_sticks.has_value() && !sticks_neutral(*pico_sticks);
    const std::array<float, 3> command = force_zero_command ?
        std::array<float, 3>{0.0f, 0.0f, 0.0f} :
        (pico_active ? command_from_pico(*pico_sticks) : command_from_remote(state));
    const auto frame = make_frame(state, command);
    if (history_.size() != kLocoObservationDim) {
      throw std::runtime_error("Locomotion history was not initialized");
    }
    history_.erase(history_.begin(), history_.begin() + static_cast<std::ptrdiff_t>(kLocoFrameDim));
    history_.insert(history_.end(), frame.begin(), frame.end());
    const auto begin = Clock::now();
    std::vector<float> raw = model_.infer(history_);
    inference_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    for (float & value : raw) {
      value = std::clamp(value, -100.0f, 100.0f);
      if (!std::isfinite(value)) throw std::runtime_error("Locomotion ONNX produced a non-finite action");
    }
    last_action_ = raw;
    std::vector<float> target_ref(kDof);
    for (size_t i = 0; i < kDof; ++i) {
      target_ref[i] = raw[i] * action_scale_[i] + default_motion_.joint_pos[i];
    }
    std::vector<float> target_controller(kDof);
    for (size_t i = 0; i < kDof; ++i) target_controller[i] = target_ref[controller_from_ref_[i]];
    return target_controller;
  }

 private:
  std::array<float, 3> command_from_remote(const RobotState & state)
  {
    return command_from_user_axes(-state.lx, state.ly, -state.rx);
  }

  std::array<float, 3> command_from_pico(const StickInput & sticks)
  {
    return command_from_user_axes(-sticks.left_x, sticks.left_y, -sticks.right_x);
  }

  static bool sticks_neutral(const StickInput & sticks)
  {
    return std::abs(sticks.left_x) <= kDeadZone && std::abs(sticks.left_y) <= kDeadZone &&
           std::abs(sticks.right_x) <= kDeadZone;
  }

  std::array<float, 3> command_from_user_axes(float left_x, float left_y, float right_x)
  {
    const float user_lx = std::clamp(left_x, -1.0f, 1.0f);
    const float user_ly = std::clamp(left_y, -1.0f, 1.0f);
    const float user_rx = std::clamp(right_x, -1.0f, 1.0f);
    std::array<float, 3> command{0.0f, 0.0f, 0.0f};
    if (user_ly < -kDeadZone) command[0] = user_ly * 0.8f;
    else if (user_ly > kDeadZone) command[0] = user_ly * 1.0f;
    if (user_lx < -kDeadZone) command[1] = user_lx * 1.0f;
    else if (user_lx > kDeadZone) command[1] = user_lx * 1.0f;
    if (user_rx < -kDeadZone) command[2] = user_rx * 3.14f;
    else if (user_rx > kDeadZone) command[2] = user_rx * 3.14f;
    for (size_t i = 0; i < command.size(); ++i) {
      command[i] = previous_command_[i] * kCommandSmoothing + command[i] * (1.0f - kCommandSmoothing);
    }
    previous_command_ = command;
    return command;
  }

  std::vector<float> make_frame(const RobotState & state, const std::array<float, 3> & command) const
  {
    std::vector<float> frame;
    frame.reserve(kLocoFrameDim);
    frame.push_back(state.gyro.x);
    frame.push_back(state.gyro.y);
    frame.push_back(state.gyro.z);
    const Vec3 gravity = rotate(conjugate(state.quat), {0.0f, 0.0f, -1.0f});
    frame.push_back(gravity.x);
    frame.push_back(gravity.y);
    frame.push_back(gravity.z);
    frame.insert(frame.end(), command.begin(), command.end());
    const auto q_ref = map_to_reference(state.q);
    const auto dq_ref = map_to_reference(state.dq);
    for (size_t i = 0; i < kDof; ++i) frame.push_back(q_ref[i] - default_motion_.joint_pos[i]);
    frame.insert(frame.end(), dq_ref.begin(), dq_ref.end());
    frame.insert(frame.end(), last_action_.begin(), last_action_.end());
    if (frame.size() != kLocoFrameDim) throw std::runtime_error("Locomotion observation frame size mismatch");
    for (float & value : frame) value = std::clamp(value, -100.0f, 100.0f);
    return frame;
  }

  std::vector<float> map_to_reference(const std::vector<float> & controller_values) const
  {
    std::vector<float> out(kDof);
    for (size_t i = 0; i < kDof; ++i) out[i] = controller_values[ref_from_controller_[i]];
    return out;
  }

  std::vector<float> map_from_reference(const std::vector<float> & reference_values) const
  {
    std::vector<float> out(kDof);
    for (size_t i = 0; i < kDof; ++i) out[i] = reference_values[controller_from_ref_[i]];
    return out;
  }

  static constexpr float kDeadZone = 0.2f;
  static constexpr float kCommandSmoothing = 0.0f;
  Motion default_motion_;
  LocomotionInferenceModel model_;
  std::vector<std::string> controller_names_;
  std::vector<std::string> reference_names_;
  std::vector<size_t> ref_from_controller_;
  std::vector<size_t> controller_from_ref_;
  std::vector<float> action_scale_;
  std::vector<float> kps_;
  std::vector<float> kds_;
  std::vector<float> last_action_;
  std::vector<float> history_;
  std::array<float, 3> previous_command_{0.0f, 0.0f, 0.0f};
};

class GritPolicy {
 public:
  GritPolicy(
      const YAML::Node & controller, const YAML::Node & tracking,
      const std::string & model_path, Motion motion, std::string motion_name,
      size_t motion_index, size_t motion_count, int threads)
      : motion_(std::move(motion)), default_motion_(default_motion(tracking)),
        model_(model_path, threads), motion_name_(std::move(motion_name)),
        motion_index_(motion_index), motion_count_(motion_count)
  {
    controller_names_ = yaml_string_vector(controller["policy_joint_names"], "controller.policy_joint_names");
    reference_names_ = yaml_string_vector(tracking["reference_joint_names"], "tracking.reference_joint_names");
    if (controller_names_.size() != kDof || reference_names_.size() != kDof) {
      throw std::runtime_error("Controller and reference joint lists must each contain 29 names");
    }
    ref_from_controller_ = source_indices(reference_names_, controller_names_, "controller-to-reference map");
    controller_from_ref_ = source_indices(controller_names_, reference_names_, "reference-to-controller map");
    action_scale_ = yaml_float_vector(tracking["action_scale"], "tracking.action_scale", kDof);
    encoder_offset_ = tracking["encoder_offset"] ?
        yaml_float_vector(tracking["encoder_offset"], "tracking.encoder_offset", kDof) : std::vector<float>(kDof, 0.0f);
    transition_steps_ = tracking["transition_steps"].as<size_t>(100);
    last_action_.assign(kDof, 0.0f);
    for (auto & term : history_) term.clear();
  }

  void initialize_default(const RobotState & state)
  {
    const auto anchor_joint = map_to_reference(state.q);
    segment_ = build_segment(
        default_motion_, anchor_joint, default_motion_.root_pos.front(), state.quat, transition_steps_);
    playing_motion_ = false;
    vr_streaming_ = false;
    last_action_.assign(kDof, 0.0f);
    for (auto & term : history_) term.clear();
    for (size_t i = 0; i < kHistoryLength; ++i) push_proprio(state);
  }

  void play_motion(const RobotState & state, const char * trigger = "remote A")
  {
    follow_robot_yaw_while_holding_default(state);
    const size_t index = std::min(segment_.index, segment_.frames - 1);
    std::vector<float> anchor(kDof);
    std::copy_n(segment_.joints.data() + index * kDof, kDof, anchor.data());
    segment_ = build_segment(
        motion_, anchor, segment_.positions[index], segment_.quaternions[index], transition_steps_);
    playing_motion_ = true;
    vr_streaming_ = false;
    std::cout << "[G1NpzPolicy] " << trigger << ": play NPZ " << (motion_index_ + 1) << "/"
              << motion_count_ << " from frame 0: " << motion_name_ << std::endl;
  }

  bool is_standing() const
  {
    return !playing_motion_ && !vr_streaming_ && segment_.frames > 0 && segment_.index + 1 >= segment_.frames;
  }

  bool vr_streaming() const { return vr_streaming_; }

  size_t future_horizon() const
  {
    if (segment_.frames == 0 || segment_.index >= segment_.frames) return 0;
    return segment_.frames - 1 - segment_.index;
  }

  void start_vr_session(const VrPoseFrame & first_frame)
  {
    if (segment_.frames == 0) throw std::runtime_error("Cannot start VR without a reference pose");
    const size_t index = std::min(segment_.index, segment_.frames - 1);
    vr_anchor_joint_.assign(
        segment_.joints.begin() + static_cast<std::ptrdiff_t>(index * kDof),
        segment_.joints.begin() + static_cast<std::ptrdiff_t>((index + 1) * kDof));
    vr_anchor_position_ = segment_.positions[index];
    vr_anchor_quaternion_ = segment_.quaternions[index];
    vr_source_position_ = {
        first_frame.root_position[0], first_frame.root_position[1], first_frame.root_position[2]};
    const Quat source_quaternion = normalized({
        first_frame.root_quaternion[0], first_frame.root_quaternion[1],
        first_frame.root_quaternion[2], first_frame.root_quaternion[3]});
    vr_yaw_delta_ = yaw_quat(vr_anchor_quaternion_) * conjugate(yaw_quat(source_quaternion));
    vr_transition_count_ = 0;
    playing_motion_ = false;
    vr_streaming_ = true;

    while (future_horizon() < kContextOffsets.back()) {
      append_stream_frame(vr_anchor_joint_, vr_anchor_position_, vr_anchor_quaternion_);
    }
    append_vr_frame(first_frame);
    std::cout << "[G1VrPolicy] PICO A: VR tracking started; transition_steps="
              << transition_steps_ << std::endl;
  }

  bool append_vr_frame(const VrPoseFrame & frame)
  {
    if (!vr_streaming_) return false;
    constexpr size_t high_watermark = 16;
    if (future_horizon() >= high_watermark) return false;

    const Vec3 source_position{
        frame.root_position[0], frame.root_position[1], frame.root_position[2]};
    const Quat source_quaternion = normalized({
        frame.root_quaternion[0], frame.root_quaternion[1],
        frame.root_quaternion[2], frame.root_quaternion[3]});
    Vec3 aligned_position = rotate(vr_yaw_delta_, source_position - vr_source_position_) + vr_anchor_position_;
    Quat aligned_quaternion = vr_yaw_delta_ * source_quaternion;
    std::vector<float> aligned_joint(frame.joint_position.begin(), frame.joint_position.end());

    if (vr_transition_count_ < transition_steps_) {
      ++vr_transition_count_;
      const float alpha = static_cast<float>(vr_transition_count_) /
          static_cast<float>(std::max<size_t>(1, transition_steps_));
      for (size_t i = 0; i < kDof; ++i) {
        aligned_joint[i] = vr_anchor_joint_[i] * (1.0f - alpha) + aligned_joint[i] * alpha;
      }
      aligned_position = vr_anchor_position_ * (1.0f - alpha) + aligned_position * alpha;
      aligned_quaternion = slerp(vr_anchor_quaternion_, aligned_quaternion, alpha);
    }

    append_stream_frame(aligned_joint, aligned_position, aligned_quaternion);
    return true;
  }

  void stop_vr(const char * trigger)
  {
    if (!vr_streaming_) return;
    vr_streaming_ = false;
    return_default(trigger);
  }

  void select_motion(Motion motion, std::string motion_name, size_t motion_index, size_t motion_count)
  {
    motion_ = std::move(motion);
    motion_name_ = std::move(motion_name);
    motion_index_ = motion_index;
    motion_count_ = motion_count;
    std::cout << "[G1DualPolicy] selected motion " << (motion_index_ + 1) << "/"
              << motion_count_ << ": " << motion_name_ << std::endl;
  }

  void return_default(const char * trigger = "remote START")
  {
    const size_t index = std::min(segment_.index, segment_.frames - 1);
    std::vector<float> anchor(kDof);
    std::copy_n(segment_.joints.data() + index * kDof, kDof, anchor.data());
    segment_ = build_segment(
        default_motion_, anchor, segment_.positions[index], segment_.quaternions[index], transition_steps_);
    playing_motion_ = false;
    vr_streaming_ = false;
    std::cout << "[G1NpzPolicy] " << trigger << ": return to default pose" << std::endl;
  }

  size_t transition_steps() const { return transition_steps_; }

  std::vector<float> step(const RobotState & state, double & inference_ms)
  {
    if (segment_.index + 1 < segment_.frames) {
      ++segment_.index;
    } else if (playing_motion_) {
      return_default("NPZ complete");
    }
    follow_robot_yaw_while_holding_default(state);
    push_proprio(state);
    std::vector<float> reference(kContextFrames * kFeatureDim);
    for (size_t i = 0; i < kContextFrames; ++i) {
      const size_t index = std::min(segment_.index + kContextOffsets[i], segment_.frames - 1);
      std::copy_n(
          segment_.features.data() + index * kFeatureDim, kFeatureDim,
          reference.data() + i * kFeatureDim);
    }
    std::vector<float> proprio;
    proprio.reserve(kHistoryLength * kProprioDim);
    for (const auto & term : history_) proprio.insert(proprio.end(), term.begin(), term.end());
    const auto begin = Clock::now();
    std::vector<float> raw = model_.infer(reference, proprio);
    inference_ms = std::chrono::duration<double, std::milli>(Clock::now() - begin).count();
    last_action_ = raw;
    std::vector<float> target_ref(kDof);
    for (size_t i = 0; i < kDof; ++i) {
      target_ref[i] = raw[i] * action_scale_[i] + default_motion_.joint_pos[i] - encoder_offset_[i];
      if (!std::isfinite(target_ref[i])) throw std::runtime_error("ONNX produced a non-finite action");
    }
    std::vector<float> target_controller(kDof);
    for (size_t i = 0; i < kDof; ++i) target_controller[i] = target_ref[controller_from_ref_[i]];
    return target_controller;
  }

 private:
  void append_stream_frame(
      const std::vector<float> & joints, const Vec3 & position, const Quat & quaternion)
  {
    if (joints.size() != kDof) throw std::runtime_error("VR joint frame must contain 29 values");
    constexpr size_t keep_history = 32;
    if (segment_.index > keep_history) {
      const size_t drop = segment_.index - keep_history;
      segment_.joints.erase(
          segment_.joints.begin(), segment_.joints.begin() + static_cast<std::ptrdiff_t>(drop * kDof));
      segment_.positions.erase(
          segment_.positions.begin(), segment_.positions.begin() + static_cast<std::ptrdiff_t>(drop));
      segment_.quaternions.erase(
          segment_.quaternions.begin(), segment_.quaternions.begin() + static_cast<std::ptrdiff_t>(drop));
      segment_.frames -= drop;
      segment_.index -= drop;
    }
    segment_.joints.insert(segment_.joints.end(), joints.begin(), joints.end());
    segment_.positions.push_back(position);
    segment_.quaternions.push_back(normalized(quaternion));
    ++segment_.frames;
    segment_.features = finite_difference_features(
        segment_.joints, segment_.positions, segment_.quaternions, 50.0f);
  }

  void follow_robot_yaw_while_holding_default(const RobotState & state)
  {
    if (playing_motion_ || vr_streaming_ || segment_.frames == 0 || segment_.index + 1 < segment_.frames) {
      return;
    }

    const size_t index = segment_.frames - 1;
    segment_.quaternions[index] = replace_yaw(segment_.quaternions[index], state.quat);

    float * feature = segment_.features.data() + index * kFeatureDim;
    std::fill_n(feature + kDof, kDof, 0.0f);
    const auto orientation = rot6d(segment_.quaternions[index]);
    std::copy(orientation.begin(), orientation.end(), feature + 58);
    std::fill_n(feature + 64, 6, 0.0f);
  }

  std::vector<float> map_to_reference(const std::vector<float> & controller_values) const
  {
    std::vector<float> out(kDof);
    for (size_t i = 0; i < kDof; ++i) out[i] = controller_values[ref_from_controller_[i]];
    return out;
  }

  void append_history(size_t term, const float * values, size_t count)
  {
    auto & history = history_[term];
    if (history.size() == kHistoryLength * count) history.erase(history.begin(), history.begin() + count);
    history.insert(history.end(), values, values + count);
  }

  void push_proprio(const RobotState & state)
  {
    const float gyro[] = {state.gyro.x, state.gyro.y, state.gyro.z};
    append_history(0, gyro, 3);
    const Vec3 gravity = rotate(conjugate(state.quat), {0.0f, 0.0f, -1.0f});
    const float gravity_values[] = {gravity.x, gravity.y, gravity.z};
    append_history(1, gravity_values, 3);
    const size_t ref_index = std::min(segment_.index, segment_.frames - 1);
    const Quat relative = conjugate(state.quat) * segment_.quaternions[ref_index];
    const auto orientation = rot6d(relative);
    append_history(2, orientation.data(), orientation.size());
    auto q_ref = map_to_reference(state.q);
    auto dq_ref = map_to_reference(state.dq);
    for (size_t i = 0; i < kDof; ++i) q_ref[i] -= default_motion_.joint_pos[i];
    append_history(3, q_ref.data(), q_ref.size());
    append_history(4, dq_ref.data(), dq_ref.size());
    append_history(5, last_action_.data(), last_action_.size());
  }

  Motion motion_;
  Motion default_motion_;
  InferenceModel model_;
  std::vector<std::string> controller_names_;
  std::vector<std::string> reference_names_;
  std::vector<size_t> ref_from_controller_;
  std::vector<size_t> controller_from_ref_;
  std::vector<float> action_scale_;
  std::vector<float> encoder_offset_;
  std::vector<float> last_action_;
  std::array<std::vector<float>, 6> history_;
  size_t transition_steps_ = 100;
  ReferenceSegment segment_;
  bool playing_motion_ = false;
  bool vr_streaming_ = false;
  std::vector<float> vr_anchor_joint_;
  Vec3 vr_anchor_position_;
  Quat vr_anchor_quaternion_;
  Vec3 vr_source_position_;
  Quat vr_yaw_delta_;
  size_t vr_transition_count_ = 0;
  std::string motion_name_;
  size_t motion_index_ = 0;
  size_t motion_count_ = 1;
};

struct Options {
  std::filesystem::path controller_config;
  std::filesystem::path tracking_config;
  std::filesystem::path motion_file;
  std::filesystem::path model_path;
  std::filesystem::path locomotion_model_path;
  std::string motion_source = "npz";
  std::string vr_server_host;
  int vr_request_port = 28701;
  int vr_reply_port = 28702;
  int vr_control_port = 28703;
  int vr_timeout_ms = 300;
  int inference_threads = 4;
  bool validate_only = false;
};

Options parse_options(int argc, char ** argv)
{
  Options out;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    auto value = [&](const std::string & name) {
      if (i + 1 >= argc) throw std::runtime_error("Missing value for " + name);
      return std::string(argv[++i]);
    };
    if (arg == "--controller-config") out.controller_config = value(arg);
    else if (arg == "--tracking-config") out.tracking_config = value(arg);
    else if (arg == "--motion-file" || arg == "--motion-path") out.motion_file = value(arg);
    else if (arg == "--motion-source") out.motion_source = value(arg);
    else if (arg == "--vr-server") out.vr_server_host = value(arg);
    else if (arg == "--vr-request-port") out.vr_request_port = std::stoi(value(arg));
    else if (arg == "--vr-reply-port") out.vr_reply_port = std::stoi(value(arg));
    else if (arg == "--vr-control-port") out.vr_control_port = std::stoi(value(arg));
    else if (arg == "--vr-timeout-ms") out.vr_timeout_ms = std::stoi(value(arg));
    else if (arg == "--policy-path") out.model_path = value(arg);
    else if (arg == "--locomotion-policy") out.locomotion_model_path = value(arg);
    else if (arg == "--inference-threads") out.inference_threads = std::stoi(value(arg));
    else if (arg == "--validate-only") out.validate_only = true;
    else if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: g1_npz_policy --controller-config PATH --tracking-config PATH "
                   "--locomotion-policy PATH [--motion-source npz|vr|vr_npz] "
                   "[--motion-file FILE_OR_DIR] [--vr-server HOST] [--policy-path PATH] "
                   "[--vr-timeout-ms N] [--inference-threads N] [--validate-only]\n";
      std::exit(0);
    } else throw std::runtime_error("Unknown argument: " + arg);
  }
  std::transform(out.motion_source.begin(), out.motion_source.end(), out.motion_source.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (out.controller_config.empty() || out.tracking_config.empty() || out.locomotion_model_path.empty()) {
    throw std::runtime_error(
        "--controller-config, --tracking-config and --locomotion-policy are required");
  }
  if (out.motion_source != "npz" && out.motion_source != "vr" && out.motion_source != "vr_npz") {
    throw std::runtime_error("--motion-source must be npz, vr or vr_npz");
  }
  if (out.motion_source != "vr" && out.motion_file.empty()) {
    throw std::runtime_error("--motion-file/--motion-path is required for NPZ mode");
  }
  if (out.motion_source != "npz" && out.vr_server_host.empty()) {
    throw std::runtime_error("--vr-server is required for VR mode");
  }
  const auto valid_port = [](int port) { return port > 0 && port <= 65535; };
  if (!valid_port(out.vr_request_port) || !valid_port(out.vr_reply_port) || !valid_port(out.vr_control_port)) {
    throw std::runtime_error("VR ports must be in the range 1..65535");
  }
  if (out.vr_timeout_ms < 100) throw std::runtime_error("--vr-timeout-ms must be at least 100");
  if (out.inference_threads < 1) throw std::runtime_error("--inference-threads must be positive");
  return out;
}

}  // namespace
}  // namespace grit_onboard

int main(int argc, char ** argv)
{
  using namespace grit_onboard;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    Options options = parse_options(argc, argv);
    options.controller_config = std::filesystem::weakly_canonical(options.controller_config);
    options.tracking_config = std::filesystem::weakly_canonical(options.tracking_config);
    if (!options.motion_file.empty()) {
      options.motion_file = std::filesystem::weakly_canonical(options.motion_file);
    }
    options.locomotion_model_path = std::filesystem::weakly_canonical(options.locomotion_model_path);
    const YAML::Node controller = YAML::LoadFile(options.controller_config.string());
    const YAML::Node tracking = YAML::LoadFile(options.tracking_config.string());
    if (options.model_path.empty()) {
      options.model_path = resolve_path(
          yaml_required<std::string>(tracking["policy_path"], "tracking.policy_path"),
          options.tracking_config);
    } else {
      options.model_path = std::filesystem::weakly_canonical(options.model_path);
    }
    std::vector<std::filesystem::path> motion_paths;
    size_t selected_motion_index = 0;
    Motion motion;
    std::string motion_name = "VR stream";
    const bool npz_enabled = options.motion_source != "vr";
    const bool vr_enabled = options.motion_source != "npz";
    if (npz_enabled) {
      motion_paths = discover_motion_paths(options.motion_file);
      motion = load_motion(motion_paths.front().string());
      motion_name = motion_paths.front().filename().string();
    } else {
      motion = default_motion(tracking);
    }
    GritPolicy grit_policy(
        controller, tracking, options.model_path.string(), std::move(motion),
        motion_name, selected_motion_index, motion_paths.size(),
        options.inference_threads);
    LocomotionPolicy locomotion_policy(
        controller, tracking, options.locomotion_model_path.string(), options.inference_threads);
    if (options.validate_only) {
      for (size_t i = 1; i < motion_paths.size(); ++i) {
        load_motion(motion_paths[i].string());
      }
      std::cout << "[G1DualPolicy] both policies and " << options.motion_source
                << " configuration passed validation; hardware transport was not opened" << std::endl;
      return 0;
    }
    std::unique_ptr<VrMotionClient> vr_client;
    if (vr_enabled) {
      const auto endpoint = [&](int port) {
        return "tcp://" + options.vr_server_host + ":" + std::to_string(port);
      };
      vr_client = std::make_unique<VrMotionClient>(VrMotionClientConfig{
          endpoint(options.vr_request_port), endpoint(options.vr_reply_port),
          endpoint(options.vr_control_port)});
    }
    RobotTransport transport(controller["udp"]);
    const auto init_q = yaml_float_vector(controller["init_qpos"], "controller.init_qpos", kDof);
    const auto whole_body_kp = yaml_float_vector(controller["kps"], "controller.kps", kDof);
    const auto whole_body_kd = yaml_float_vector(controller["kds"], "controller.kds", kDof);
    const auto & locomotion_kp = locomotion_policy.kps();
    const auto & locomotion_kd = locomotion_policy.kds();
    const std::vector<float> zeros(kDof, 0.0f);
    const std::vector<float> damping(kDof, 8.0f);
    RobotState state;
    const bool combined_mode = vr_client && !motion_paths.empty();
    if (combined_mode) {
      std::cout << "[G1DualPolicy] waiting for bridge state; remote START=locomotion, "
                   "remote B or PICO B cycles Loco/VR/NPZ, A starts the active VR/NPZ mode, "
                   "PICO X/remote START=stand, PICO Y/remote X=emergency stop" << std::endl;
    } else if (vr_client) {
      std::cout << "[G1DualPolicy] waiting for bridge state; remote START=locomotion, "
                   "remote B or PICO B=switch Loco/WBC, PICO A=start VR, "
                   "PICO X=return to WBC stand, PICO Y/remote X=emergency stop" << std::endl;
    } else {
      std::cout << "[G1DualPolicy] waiting for bridge state; remote START=locomotion/WBC stand, "
                   "B=switch policy, A=play NPZ, left/right=select NPZ while standing, "
                   "X=emergency stop" << std::endl;
    }
    while (!g_stop.load() && !transport.receive(state, 1000)) {
      std::cout << "[G1NpzPolicy] still waiting for bridge state" << std::endl;
    }
    enum class WholeBodyMode {
      VrTracking,
      NpzPlayback
    };
    enum class Mode {
      ZeroTorque,
      MoveToDefault,
      LocomotionBlend,
      Locomotion,
      LocomotionStopping,
      WholeBodyBlend,
      WholeBody,
      WholeBodySwitching,
      WholeBodyStopping
    };
    Mode mode = Mode::ZeroTorque;
    WholeBodyMode whole_body_mode = vr_client ?
        WholeBodyMode::VrTracking : WholeBodyMode::NpzPlayback;
    std::vector<float> move_start;
    std::vector<float> blend_start(kDof, 0.0f);
    std::vector<float> blend_kp_start = locomotion_kp;
    std::vector<float> blend_kd_start = locomotion_kd;
    std::vector<float> last_target(kDof, 0.0f);
    size_t move_step = 0;
    size_t mode_step = 0;
    constexpr size_t move_steps = 100;
    constexpr size_t blend_steps = 50;
    constexpr size_t locomotion_stop_steps = 50;
    bool previous_start = false;
    bool previous_a = false;
    bool previous_b = false;
    bool previous_left = false;
    bool previous_right = false;
    bool pico_motion_switch_armed = false;
    bool neutral_wait_logged = false;
    bool vr_start_pending = false;
    bool vr_request_inflight = false;
    uint64_t vr_request_time_ns = 0;
    bool vr_timeout_logged = false;
    std::optional<size_t> pending_motion_index;
    std::future<Motion> pending_motion_future;
    std::optional<Motion> loaded_motion;
    uint64_t count = 0;
    double infer_sum = 0.0;
    double infer_max = 0.0;
    double delay_sum = 0.0;
    double delay_max = 0.0;
    auto last_log = Clock::now();
    bool damping_requested = false;
    const auto send_damping = [&]() {
      transport.send(zeros, zeros, zeros, damping, 0, state.receive_time_ns);
    };
    const auto blend_values = [&](const std::vector<float> & start, const std::vector<float> & target, size_t step) {
      const float alpha = std::min(
          1.0f, static_cast<float>(step + 1) / static_cast<float>(blend_steps));
      std::vector<float> blended(kDof);
      for (size_t i = 0; i < kDof; ++i) {
        blended[i] = start[i] * (1.0f - alpha) + target[i] * alpha;
      }
      return blended;
    };
    const auto mode_name = [&]() {
      switch (mode) {
        case Mode::ZeroTorque: return "zero_torque";
        case Mode::MoveToDefault: return "move_to_default";
        case Mode::LocomotionBlend: return "locomotion_blend";
        case Mode::Locomotion: return "locomotion";
        case Mode::LocomotionStopping: return "locomotion_stopping";
        case Mode::WholeBodyBlend: return "whole_body_blend";
        case Mode::WholeBody: return whole_body_mode == WholeBodyMode::VrTracking ?
            "vr_tracking" : "npz_playback";
        case Mode::WholeBodySwitching: return "whole_body_switching";
        case Mode::WholeBodyStopping: return "whole_body_stopping";
      }
      return "unknown";
    };
    const auto request_motion = [&](int direction) {
      if (motion_paths.empty()) {
        std::cout << "[G1DualPolicy] NPZ selection is unavailable in VR mode" << std::endl;
        return;
      }
      if (motion_paths.size() == 1) {
        std::cout << "[G1DualPolicy] playlist has one motion; left/right ignored" << std::endl;
        return;
      }
      if (pending_motion_index.has_value()) {
        std::cout << "[G1DualPolicy] motion selection ignored while another NPZ is loading" << std::endl;
        return;
      }
      const size_t next_index = direction > 0 ?
          (selected_motion_index + 1) % motion_paths.size() :
          (selected_motion_index + motion_paths.size() - 1) % motion_paths.size();
      const auto path = motion_paths[next_index];
      pending_motion_index = next_index;
      pending_motion_future = std::async(std::launch::async, [path]() {
        return load_motion(path.string());
      });
      std::cout << "[G1DualPolicy] loading motion " << (next_index + 1) << "/"
                << motion_paths.size() << " in background: " << path.filename().string() << std::endl;
    };
    try {
      while (!g_stop.load()) {
        if (!transport.receive(state, 1000)) {
          if (g_stop.load()) break;
          throw std::runtime_error("No bridge state for 1 s; damping requested");
        }
        if (state.x) {
          std::cout << "[G1NpzPolicy] remote X: damping and exit" << std::endl;
          damping_requested = true;
          break;
        }
        const bool start_rise = state.start && !previous_start;
        const bool a_rise = state.a && !previous_a;
        const bool b_rise = state.b && !previous_b;
        const bool left_rise = state.left && !previous_left;
        const bool right_rise = state.right && !previous_right;
        previous_start = state.start;
        previous_a = state.a;
        previous_b = state.b;
        previous_left = state.left;
        previous_right = state.right;

        VrOperatorInput vr_input;
        std::optional<LocomotionPolicy::StickInput> pico_sticks;
        std::vector<VrPoseFrame> vr_frames;
        int pico_motion_direction = 0;
        if (vr_client) {
          vr_input = vr_client->poll_operator_input();
          if (vr_input.sticks_valid) {
            pico_sticks = LocomotionPolicy::StickInput{
                vr_input.left_x, vr_input.left_y, vr_input.right_x};
            const float selection_axis = std::abs(vr_input.left_x) >= std::abs(vr_input.right_x) ?
                vr_input.left_x : vr_input.right_x;
            constexpr float selection_engage = 0.75f;
            constexpr float selection_release = 0.25f;
            if (std::abs(selection_axis) <= selection_release) {
              pico_motion_switch_armed = true;
            } else if (pico_motion_switch_armed && std::abs(selection_axis) >= selection_engage) {
              pico_motion_switch_armed = false;
              if (mode == Mode::WholeBody && whole_body_mode == WholeBodyMode::NpzPlayback) {
                pico_motion_direction = selection_axis > 0.0f ? 1 : -1;
              }
            }
          }
          vr_frames = vr_client->poll_pose_frames();
          if (!vr_frames.empty()) vr_request_inflight = false;
          if (vr_input.stop_control) {
            std::cout << "[G1VrPolicy] PICO Y: damping and exit" << std::endl;
            damping_requested = true;
            break;
          }
        }

        if (pending_motion_index.has_value() && pending_motion_future.valid() &&
            pending_motion_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
          try {
            loaded_motion.emplace(pending_motion_future.get());
          } catch (const std::exception & error) {
            std::cerr << "[G1DualPolicy] failed to load selected motion: " << error.what() << std::endl;
            pending_motion_index.reset();
          }
        }
        const bool npz_selection_ready =
            mode == Mode::WholeBody && whole_body_mode == WholeBodyMode::NpzPlayback &&
            grit_policy.is_standing();
        if (pending_motion_index.has_value() && loaded_motion.has_value() && npz_selection_ready) {
          selected_motion_index = *pending_motion_index;
          grit_policy.select_motion(
              std::move(*loaded_motion), motion_paths[selected_motion_index].filename().string(),
              selected_motion_index, motion_paths.size());
          transport.announce(motion_voice_label(motion_paths[selected_motion_index]));
          loaded_motion.reset();
          pending_motion_index.reset();
        }
        if ((left_rise || right_rise) && !npz_selection_ready) {
          std::cout << "[G1DualPolicy] left/right ignored: motion selection is allowed only in "
                       "stable NPZ standing mode" << std::endl;
        }
        if (pico_motion_direction != 0 && !npz_selection_ready) {
          std::cout << "[G1DualPolicy] PICO stick ignored: motion selection is allowed only in "
                       "stable NPZ standing mode" << std::endl;
        }
        if (mode == Mode::ZeroTorque) {
          transport.send(zeros, zeros, zeros, zeros, 0, state.receive_time_ns);
          if (state.start) {
            move_start = state.q;
            move_step = 0;
            mode = Mode::MoveToDefault;
            std::cout << "[G1DualPolicy] remote START: moving to the locomotion default pose" << std::endl;
          }
          continue;
        }
        if (mode == Mode::MoveToDefault) {
          const float alpha = static_cast<float>(move_step + 1) / static_cast<float>(move_steps);
          std::vector<float> target(kDof);
          for (size_t i = 0; i < kDof; ++i) target[i] = move_start[i] * (1.0f - alpha) + init_q[i] * alpha;
          transport.send(target, zeros, locomotion_kp, locomotion_kd, 1, state.receive_time_ns);
          last_target = target;
          if (++move_step >= move_steps) {
            locomotion_policy.reset(state);
            blend_start = last_target;
            mode_step = 0;
            mode = Mode::LocomotionBlend;
            last_log = Clock::now();
            std::cout << "[G1DualPolicy] locomotion policy is taking control at zero velocity" << std::endl;
          }
          continue;
        }

        double inference_ms = 0.0;
        std::vector<float> target;
        std::vector<float> command_kp;
        std::vector<float> command_kd;
        if (mode == Mode::LocomotionBlend) {
          const auto policy_target = locomotion_policy.step(state, true, pico_sticks, inference_ms);
          target = mode_step < blend_steps ? blend_values(blend_start, policy_target, mode_step) : policy_target;
          command_kp = mode_step < blend_steps ?
              blend_values(blend_kp_start, locomotion_kp, mode_step) : locomotion_kp;
          command_kd = mode_step < blend_steps ?
              blend_values(blend_kd_start, locomotion_kd, mode_step) : locomotion_kd;
          if (mode_step < blend_steps) ++mode_step;
          if (mode_step >= blend_steps && locomotion_policy.sticks_neutral(state, pico_sticks)) {
            mode = Mode::Locomotion;
            neutral_wait_logged = false;
            transport.announce_mode(vr_client ? "loco mode" : "Loco mode", "loco");
            std::cout << "[G1DualPolicy] locomotion active; use the G1/PICO sticks for vx/vy/yaw, "
                         "press remote B or PICO B for whole-body control" << std::endl;
          } else if (mode_step >= blend_steps && !neutral_wait_logged) {
            neutral_wait_logged = true;
            std::cout << "[G1DualPolicy] waiting for centered G1 and PICO locomotion sticks before enabling motion"
                      << std::endl;
          }
        } else if (mode == Mode::Locomotion) {
          if (b_rise || (vr_client && (vr_input.toggle_mode || vr_input.activate_or_default))) {
            if (combined_mode) whole_body_mode = WholeBodyMode::VrTracking;
            mode = Mode::LocomotionStopping;
            mode_step = 0;
            const char * trigger = b_rise ? "remote B" :
                (vr_input.toggle_mode ? "PICO B" : "PICO X");
            std::cout << "[G1DualPolicy] " << trigger
                      << ": stopping locomotion before whole-body handoff" << std::endl;
          }
          target = locomotion_policy.step(
              state, mode == Mode::LocomotionStopping, pico_sticks, inference_ms);
          command_kp = locomotion_kp;
          command_kd = locomotion_kd;
        } else if (mode == Mode::LocomotionStopping) {
          target = locomotion_policy.step(state, true, pico_sticks, inference_ms);
          command_kp = locomotion_kp;
          command_kd = locomotion_kd;
          if (++mode_step >= locomotion_stop_steps) {
            grit_policy.initialize_default(state);
            blend_start = target;
            blend_kp_start = locomotion_kp;
            blend_kd_start = locomotion_kd;
            mode_step = 0;
            mode = Mode::WholeBodyBlend;
            std::cout << "[G1DualPolicy] whole-body policy is taking control in standing mode" << std::endl;
          }
        } else if (mode == Mode::WholeBodyBlend) {
          const auto policy_target = grit_policy.step(state, inference_ms);
          target = blend_values(blend_start, policy_target, mode_step);
          command_kp = blend_values(blend_kp_start, whole_body_kp, mode_step);
          command_kd = blend_values(blend_kd_start, whole_body_kd, mode_step);
          if (++mode_step >= grit_policy.transition_steps() + 25) {
            mode = Mode::WholeBody;
            transport.announce_mode(
                whole_body_mode == WholeBodyMode::VrTracking ? "VR Tracking mode" : "NPZ mode",
                whole_body_mode == WholeBodyMode::VrTracking ? "vr" : "npz");
            if (whole_body_mode == WholeBodyMode::VrTracking) {
              std::cout << "[G1DualPolicy] whole-body standing active; PICO A starts VR, "
                           "PICO X/remote START returns to stand, remote B or PICO B "
                        << (combined_mode ? "switches to NPZ" : "returns to locomotion")
                        << std::endl;
            } else {
              std::cout << "[G1DualPolicy] whole-body standing active; A plays NPZ, START returns to stand, "
                           "B returns to locomotion" << std::endl;
            }
          }
        } else if (mode == Mode::WholeBody) {
          const bool vr_mode = whole_body_mode == WholeBodyMode::VrTracking;
          if (b_rise || (vr_client && vr_input.toggle_mode)) {
            const char * trigger = b_rise ? "remote B" : "PICO B";
            if (vr_mode) {
              vr_start_pending = false;
              vr_request_inflight = false;
              if (grit_policy.vr_streaming()) {
                grit_policy.stop_vr(trigger);
              } else {
                grit_policy.return_default(trigger);
              }
            } else {
              grit_policy.return_default(trigger);
            }
            mode_step = 0;
            if (combined_mode && vr_mode) {
              whole_body_mode = WholeBodyMode::NpzPlayback;
              mode = Mode::WholeBodySwitching;
              std::cout << "[G1DualPolicy] " << trigger
                        << ": returning to stand before NPZ mode" << std::endl;
            } else {
              mode = Mode::WholeBodyStopping;
              std::cout << "[G1DualPolicy] " << trigger
                        << ": returning to stand before locomotion handoff" << std::endl;
            }
          } else if (start_rise) {
            if (vr_mode) {
              vr_start_pending = false;
              vr_request_inflight = false;
              if (grit_policy.vr_streaming()) {
                grit_policy.stop_vr("remote START");
              } else {
                grit_policy.return_default();
              }
            } else {
              grit_policy.return_default();
            }
          } else if (vr_client && vr_input.activate_or_default) {
            vr_start_pending = false;
            vr_request_inflight = false;
            if (vr_mode && grit_policy.vr_streaming()) {
              grit_policy.stop_vr("PICO X");
            } else {
              grit_policy.return_default("PICO X");
            }
          } else if (!vr_mode && left_rise && right_rise) {
            std::cout << "[G1DualPolicy] simultaneous left/right ignored" << std::endl;
          } else if (!vr_mode && left_rise && grit_policy.is_standing()) {
            request_motion(-1);
          } else if (!vr_mode && right_rise && grit_policy.is_standing()) {
            request_motion(1);
          } else if (!vr_mode && pico_motion_direction != 0 && grit_policy.is_standing()) {
            request_motion(pico_motion_direction);
          } else if (!vr_mode && (a_rise || (vr_client && vr_input.start_tracking))) {
            const char * trigger = a_rise ? "remote A" : "PICO A";
            if (pending_motion_index.has_value()) {
              std::cout << "[G1DualPolicy] " << trigger
                        << " ignored until the selected NPZ finishes loading" << std::endl;
            } else {
              grit_policy.play_motion(state, trigger);
            }
          }

          if (mode == Mode::WholeBody && vr_mode && vr_client) {
            if (vr_input.start_tracking) {
              vr_start_pending = true;
              vr_request_inflight = false;
              vr_timeout_logged = false;
              if (grit_policy.vr_streaming()) grit_policy.stop_vr("PICO A restart");
              std::cout << "[G1VrPolicy] PICO A: requesting a fresh VR alignment frame" << std::endl;
            }

            for (const auto & frame : vr_frames) {
              if (frame.starts_session) {
                if (!vr_start_pending) {
                  std::cout << "[G1VrPolicy] ignored delayed VR start reply" << std::endl;
                  continue;
                }
                grit_policy.start_vr_session(frame);
                vr_start_pending = false;
                vr_timeout_logged = false;
              } else if (vr_start_pending || !grit_policy.vr_streaming()) {
                continue;
              } else {
                grit_policy.append_vr_frame(frame);
              }
            }

            const uint64_t current_ns = now_ns();
            const uint64_t last_pose_ns = vr_client->last_pose_receive_time_ns();
            if (grit_policy.vr_streaming() && last_pose_ns > 0 &&
                current_ns - last_pose_ns > static_cast<uint64_t>(options.vr_timeout_ms) * 1000000ULL) {
              if (!vr_timeout_logged) {
                std::cerr << "[G1VrPolicy] VR pose timeout after " << options.vr_timeout_ms
                          << " ms; returning to WBC stand" << std::endl;
                vr_timeout_logged = true;
              }
              vr_start_pending = false;
              vr_request_inflight = false;
              grit_policy.stop_vr("VR pose timeout");
            }

            const bool needs_pose = vr_start_pending ||
                (grit_policy.vr_streaming() && grit_policy.future_horizon() <= kContextOffsets.back());
            constexpr uint64_t request_retry_ns = 100000000ULL;
            if (needs_pose && (!vr_request_inflight || current_ns - vr_request_time_ns >= request_retry_ns)) {
              if (vr_client->request_pose(vr_start_pending)) {
                vr_request_inflight = true;
                vr_request_time_ns = current_ns;
              }
            }
          }
          target = grit_policy.step(state, inference_ms);
          command_kp = whole_body_kp;
          command_kd = whole_body_kd;
        } else if (mode == Mode::WholeBodySwitching) {
          target = grit_policy.step(state, inference_ms);
          command_kp = whole_body_kp;
          command_kd = whole_body_kd;
          if (grit_policy.is_standing()) {
            mode = Mode::WholeBody;
            transport.announce_mode("NPZ mode", "npz");
            std::cout << "[G1DualPolicy] NPZ standing active; A plays NPZ, left/right selects motion, "
                         "START returns to stand, remote B or PICO B returns to locomotion"
                      << std::endl;
          }
        } else if (mode == Mode::WholeBodyStopping) {
          target = grit_policy.step(state, inference_ms);
          command_kp = whole_body_kp;
          command_kd = whole_body_kd;
          if (++mode_step >= grit_policy.transition_steps() + 25) {
            locomotion_policy.reset(state);
            blend_start = target;
            blend_kp_start = whole_body_kp;
            blend_kd_start = whole_body_kd;
            mode_step = 0;
            neutral_wait_logged = false;
            mode = Mode::LocomotionBlend;
            std::cout << "[G1DualPolicy] locomotion policy is taking control at zero velocity" << std::endl;
          }
        }
        transport.send(target, zeros, command_kp, command_kd, 1, state.receive_time_ns);
        last_target = target;
        const double delay_ms = static_cast<double>(now_ns() - state.receive_time_ns) * 1e-6;
        ++count;
        infer_sum += inference_ms;
        infer_max = std::max(infer_max, inference_ms);
        delay_sum += delay_ms;
        delay_max = std::max(delay_max, delay_ms);
        if (Clock::now() - last_log >= std::chrono::seconds(1)) {
          std::cout << std::fixed << std::setprecision(3)
                    << "[G1DualPolicy] mode=" << mode_name() << " rate=" << count
                    << " Hz inference_ms mean=" << infer_sum / count
                    << " max=" << infer_max << " state_to_cmd_ms mean=" << delay_sum / count
                    << " max=" << delay_max << std::endl;
          count = 0;
          infer_sum = infer_max = delay_sum = delay_max = 0.0;
          last_log = Clock::now();
        }
      }
    } catch (...) {
      try {
        send_damping();
      } catch (const std::exception & error) {
        std::cerr << "[G1NpzPolicy] Failed to send damping after error: " << error.what() << std::endl;
      }
      throw;
    }
    if (damping_requested || g_stop.load()) {
      send_damping();
    }
  } catch (const std::exception & error) {
    std::cerr << "[G1NpzPolicy] Fatal: " << error.what() << std::endl;
    return 1;
  }
  return 0;
}
