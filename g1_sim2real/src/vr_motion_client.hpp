#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace grit_onboard {

constexpr std::size_t kVrJointCount = 29;

struct VrPoseFrame {
  std::array<float, 3> root_position{};
  // Scalar-first quaternion, matching the GRIT reference contract.
  std::array<float, 4> root_quaternion{1.0f, 0.0f, 0.0f, 0.0f};
  std::array<float, kVrJointCount> joint_position{};
  bool starts_session = false;
  std::uint64_t receive_time_ns = 0;
};

struct VrOperatorInput {
  bool activate_or_default = false;  // PICO X
  bool start_tracking = false;       // PICO A
  bool toggle_mode = false;          // PICO B
  bool stop_control = false;         // PICO Y
  bool sticks_valid = false;
  float left_x = 0.0f;
  float left_y = 0.0f;
  float right_x = 0.0f;
};

struct VrMotionClientConfig {
  std::string request_address;
  std::string reply_address;
  std::string control_address;
};

// Non-blocking adapter for the existing XRoboToolkit retarget server.
// The control loop calls poll_operator_input(), poll_pose_frames(), and request_pose();
// this module owns all ZeroMQ protocol and JSON/YAML decoding details.
class VrMotionClient {
 public:
  explicit VrMotionClient(VrMotionClientConfig config);
  ~VrMotionClient();

  VrMotionClient(const VrMotionClient &) = delete;
  VrMotionClient & operator=(const VrMotionClient &) = delete;

  VrOperatorInput poll_operator_input();
  std::vector<VrPoseFrame> poll_pose_frames();
  bool request_pose(bool starts_session);

  bool control_received() const;
  std::uint64_t last_control_receive_time_ns() const;
  std::uint64_t last_pose_receive_time_ns() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace grit_onboard
