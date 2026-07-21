#include "pololu_ros/log_client.hpp"
#include "pololu_ros/crtp_log.hpp"

namespace pololu_ros {

using bitcraze::crazyflieLinkCpp::Packet;

LogClient::LogClient(std::shared_ptr<Connection> conn, uint8_t robot_id)
    : conn_(std::move(conn)), robot_id_(robot_id) {}

LogClient::~LogClient() { stop(); }

void LogClient::start(uint8_t period_tens_ms) {
  if (running_.exchange(true)) return;  // already running
  latest_ = RobotLog{};
  latest_.robot_id = robot_id_;
  for (uint8_t id = 0; id < 4; ++id) {
    conn_->send(crtpLogStartRequest(id, period_tens_ms));
  }
  rx_ = std::thread(&LogClient::recvLoop, this);
}

void LogClient::stop() {
  if (!running_.exchange(false)) return;
  for (uint8_t id = 0; id < 4; ++id) {
    conn_->send(crtpLogStopRequest(id));
  }
  if (rx_.joinable()) rx_.join();
}

void LogClient::recvLoop() {
  while (running_.load()) {
    Packet p = conn_->receive(100);  // 100ms timeout so stop() stays responsive
    if (!crtpLogDataResponse::valid(p)) continue;

    latest_.t_ms = crtpLogDataResponse::timestampMS(p);
    auto f = [&](uint8_t i) { return crtpLogDataResponse::variableAt<float>(p, i * 4); };

    switch (crtpLogDataResponse::blockId(p)) {
      case 0:
        latest_.x = f(0); latest_.y = f(1); latest_.yaw = f(2);
        latest_.x_des = f(3); latest_.y_des = f(4); latest_.yaw_des = f(5);
        break;
      case 1:
        latest_.x_err = f(0); latest_.y_err = f(1); latest_.yaw_err = f(2);
        latest_.v_ff = f(3); latest_.w_ff = f(4);
        break;
      case 2:
        latest_.duty_l = f(0); latest_.duty_r = f(1);
        latest_.omega_l_cmd = f(2); latest_.omega_r_cmd = f(3);
        latest_.omega_l_meas = f(4); latest_.omega_r_meas = f(5);
        break;
      case 3:
        latest_.mode = f(0); latest_.running = f(1);
        break;
      default:
        continue;
    }
    if (on_snapshot) on_snapshot(latest_);
  }
}

} // namespace pololu_ros
