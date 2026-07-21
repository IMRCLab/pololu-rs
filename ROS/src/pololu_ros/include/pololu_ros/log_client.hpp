#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

#include "crazyflieLinkCpp/Connection.h"

namespace pololu_ros {

struct RobotLog {
  uint8_t robot_id = 0;
  uint32_t t_ms = 0;
  // block 0: pose
  float x = 0, y = 0, yaw = 0, x_des = 0, y_des = 0, yaw_des = 0;
  // block 1: error
  float x_err = 0, y_err = 0, yaw_err = 0, v_ff = 0, w_ff = 0;
  // block 2: wheel
  float duty_l = 0, duty_r = 0, omega_l_cmd = 0, omega_r_cmd = 0,
        omega_l_meas = 0, omega_r_meas = 0;
  // block 3: status
  float mode = 0, running = 0;
};

// Pulls the dongle's 4 fixed log blocks over the shared radio Connection.
// Borrows the connection (single radio owner), never opens its own.
class LogClient {
public:
  using Connection = bitcraze::crazyflieLinkCpp::Connection;

  LogClient(std::shared_ptr<Connection> conn, uint8_t robot_id = 0);
  ~LogClient();

  void start(uint8_t period_tens_ms);  // 2 = 20ms = 50Hz
  void stop();

  std::function<void(const RobotLog&)> on_snapshot;

private:
  void recvLoop();

  std::shared_ptr<Connection> conn_;
  uint8_t robot_id_;
  std::atomic<bool> running_{false};
  std::thread rx_;
  RobotLog latest_;
};

} // namespace pololu_ros
