// mocap_to_robots.cpp
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>
#include <string>
#include <cmath>

#include <motion_capture_tracking_interfaces/msg/named_pose_array.hpp>

// CrazyflieBroadcaster 的公共头在 crazyflie 包中：
// 注意：不同版本可能路径略有差异，如找不到请在 crazyflie 包里 grep "CrazyflieBroadcaster"
#include <crazyflie_cpp/Crazyflie.h>

using mcti = motion_capture_tracking_interfaces::msg;

class MocapToRobotsNode : public rclcpp::Node
{
public:
  MocapToRobotsNode() : Node("mocap_to_robots")
  {
    // 参数：mocap 话题名、name→id 映射
    this->declare_parameter<std::string>("mocap_topic", "/poses");
    this->declare_parameter<std::vector<std::string>>("names", {"cf1"});
    this->declare_parameter<std::vector<int64_t>>("ids", {1});

    const auto topic = this->get_parameter("mocap_topic").as_string();
    const auto names = this->get_parameter("names").as_string_array();
    const auto ids64 = this->get_parameter("ids").as_integer_array();

    if (names.size() != ids64.size()) {
      RCLCPP_FATAL(get_logger(), "Parameter size mismatch: names(%zu) != ids(%zu)",
                   names.size(), ids64.size());
      throw std::runtime_error("names/ids size mismatch");
    }

    for (size_t i = 0; i < names.size(); ++i) {
      name_to_id_[names[i]] = static_cast<uint8_t>(ids64[i] & 0xFF);
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu name→id mappings.", name_to_id_.size());

    // Broadcaster（使用缺省构造就行；如需指定 radio，可在 crazyflie 包中看构造函数并加参数）
    broadcaster_ = std::make_shared<crazyswarm2::CrazyflieBroadcaster>();

    // 订阅 mocap 位姿
    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);
    sub_ = this->create_subscription<mcti::NamedPoseArray>(
      topic, qos,
      std::bind(&MocapToRobotsNode::poses_cb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribing mocap poses on: %s", topic.c_str());
  }

private:
  void poses_cb(const mcti::NamedPoseArray::SharedPtr msg)
  {
    using ExtPos  = crazyswarm2::CrazyflieBroadcaster::externalPosition;
    using ExtPose = crazyswarm2::CrazyflieBroadcaster::externalPose;

    std::vector<ExtPos>  data_position;
    std::vector<ExtPose> data_pose;
    data_position.reserve(msg->poses.size());
    data_pose.reserve(msg->poses.size());

    for (const auto& np : msg->poses) {
      auto it = name_to_id_.find(np.name);
      if (it == name_to_id_.end()) continue;

      const uint8_t id = it->second;
      const auto& p = np.pose.position;
      const auto& q = np.pose.orientation;

      if (std::isnan(q.w)) {
        // 仅位置
        data_position.push_back(ExtPos{
          id,
          static_cast<float>(p.x),
          static_cast<float>(p.y),
          static_cast<float>(p.z)
        });
      } else {
        // 完整位姿
        data_pose.push_back(ExtPose{
          id,
          static_cast<float>(p.x),
          static_cast<float>(p.y),
          static_cast<float>(p.z),
          static_cast<float>(q.x),
          static_cast<float>(q.y),
          static_cast<float>(q.z),
          static_cast<float>(q.w)
        });
      }
    }

    // 群发：位置-only
    if (!data_position.empty()) {
      broadcaster_->sendExternalPositions(data_position);
    }
    // 群发：位姿
    if (!data_pose.empty()) {
      broadcaster_->sendExternalPoses(data_pose);
    }
  }

  std::unordered_map<std::string, uint8_t> name_to_id_;
  rclcpp::Subscription<mcti::NamedPoseArray>::SharedPtr sub_;
  std::shared_ptr<crazyswarm2::CrazyflieBroadcaster> broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapToRobotsNode>());
  rclcpp::shutdown();
  return 0;
}
