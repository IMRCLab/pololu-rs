#include <memory>
#include <vector>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "crazyflieLinkCpp/Connection.h"
#include "PacketUtils.hpp"

using namespace bitcraze::crazyflieLinkCpp;
using std::placeholders::_1;

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
        : Node("teleop")
        , logger_(this->get_logger())
    {
        // for (int i = 0; i < 4; ++i) {
        //     subscription_[i] = this->create_subscription<sensor_msgs::msg::Joy>(
        //         "joy" + std::to_string(i), 1, std::bind(&TeleopNode::joyChanged, this, i, _1));
        // }

        subscription_[0] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy1", 1, std::bind(&TeleopNode::joyChanged1, this, _1));

        subscription_[1] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy2", 1, std::bind(&TeleopNode::joyChanged2, this, _1));

        subscription_[2] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy3", 1, std::bind(&TeleopNode::joyChanged3, this, _1));

        subscription_[3] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy4", 1, std::bind(&TeleopNode::joyChanged4, this, _1));

        this->declare_parameter("frequency", 10);
        this->get_parameter<int>("frequency", frequency_);

        this->declare_parameter("uri1", "radio://*/80/2M/E7C2C2C210?safelink=0&autoping=0");
        std::string uri1;
        this->get_parameter<std::string>("uri1", uri1);
        connection_[0] = std::make_shared<Connection>(uri1);
        std::cout << "Connection 1: " << uri1 << std::endl;         

        this->declare_parameter("uri2", "radio://*/80/2M/E7C2C2C209?safelink=0&autoping=0");
        std::string uri2;
        this->get_parameter<std::string>("uri2", uri2);
        connection_[1] = std::make_shared<Connection>(uri2);    

        this->declare_parameter("uri3", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri3;
        this->get_parameter<std::string>("uri3", uri3);
        connection_[2] = std::make_shared<Connection>(uri3);

        this->declare_parameter("uri4", "radio://*/80/2M/E7C2C2C207?safelink=0&autoping=0");
        std::string uri4;
        this->get_parameter<std::string>("uri4", uri4);
        connection_[3] = std::make_shared<Connection>(uri4);

        // declare cmd_rpy params
        declareAxis("joy1.yawrate");
        declareAxis("joy1.thrust");
        getAxis("joy1.yawrate", axes_[0].yaw);
        getAxis("joy1.thrust", axes_[0].z);

        declareAxis("joy2.yawrate");
        declareAxis("joy2.thrust");
        getAxis("joy2.yawrate", axes_[1].yaw);
        getAxis("joy2.thrust", axes_[1].z);

        declareAxis("joy3.yawrate");
        declareAxis("joy3.thrust");
        getAxis("joy3.yawrate", axes_[2].yaw);
        getAxis("joy3.thrust", axes_[2].z);

        declareAxis("joy4.yawrate");
        declareAxis("joy4.thrust");
        getAxis("joy4.yawrate", axes_[3].yaw);
        getAxis("joy4.thrust", axes_[3].z);

        dt_ = 1.0f/frequency_;
        if (frequency_ > 0) {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency_), std::bind(&TeleopNode::publish, this));
        }
    }

private:

    struct Axis
    { 
        int axis;
        float max;
        float deadband;
    };
    struct
    {
        Axis z;
        Axis yaw;
    } axes_[4];

    // Publish Function, sending tele operation command
    void publish() 
    {
        for (int i =0; i < 4; ++i) {
            connection_[i]->send(PacketUtils::cmdLegacy_Pololu_Teleop(twist_[i].linear.z, twist_[i].angular.z));
            RCLCPP_INFO(logger_, "sending");
            RCLCPP_INFO(logger_, "Twist[%d]: linear.z=%.2f, angular.z=%.2f", i, twist_[i].linear.z, twist_[i].angular.z);
        }
    }

    void joyChanged1(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        twist_[0].linear.z = getAxis(msg, axes_[0].z);
        twist_[0].angular.z = getAxis(msg, axes_[0].yaw);
    }

    void joyChanged2(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        twist_[1].linear.z = getAxis(msg, axes_[1].z);
        twist_[1].angular.z = getAxis(msg, axes_[1].yaw);
    }

    void joyChanged3(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        twist_[2].linear.z = getAxis(msg, axes_[2].z);
        twist_[2].angular.z = getAxis(msg, axes_[2].yaw);
    }

    void joyChanged4(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        twist_[3].linear.z = getAxis(msg, axes_[3].z);
        twist_[3].angular.z = getAxis(msg, axes_[3].yaw);
    }

    sensor_msgs::msg::Joy::_axes_type::value_type getAxis(const sensor_msgs::msg::Joy::SharedPtr &msg, Axis a)
    {
        if (a.axis == 0) {
            return 0;
        }

        sensor_msgs::msg::Joy::_axes_type::value_type sign = 1.0;
        if (a.axis < 0) {
            sign = -1.0;
            a.axis = -a.axis;
        }
        if ((size_t) a.axis > msg->axes.size()) {
            return 0;
        }
        auto result = sign * msg->axes[a.axis - 1]*a.max;
        if (fabs(result) > a.deadband) {
            return result;
        } else {
            return 0;
        }
    }

    void declareAxis(const std::string& name)
    {
        this->declare_parameter<int>(name + ".axis");
        this->declare_parameter<float>(name + ".max");
        this->declare_parameter<float>(name + ".deadband");
    }

    void getAxis(const std::string& name, Axis& axis)
    {
        this->get_parameter<int>(name + ".axis", axis.axis);
        this->get_parameter<float>(name + ".max", axis.max);
        this->get_parameter<float>(name + ".deadband", axis.deadband);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_[4];
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_[4];
    rclcpp::Logger logger_;
    int frequency_;
    float dt_;

    std::shared_ptr<Connection> connection_[4];
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}

