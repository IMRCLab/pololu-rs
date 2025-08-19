#include <memory>
#include <vector>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

//includes for MotionCapture tracking interface
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"

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

        //include listener that will send MoCap package about position to the robot

        subscription_[0] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy1", 1, std::bind(&TeleopNode::joyChanged1, this, _1));

        subscription_[1] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy2", 1, std::bind(&TeleopNode::joyChanged2, this, _1));

        subscription_[2] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy3", 1, std::bind(&TeleopNode::joyChanged3, this, _1));

        subscription_[3] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy4", 1, std::bind(&TeleopNode::joyChanged4, this, _1));

        mocap_subscription = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
            "poses", 
            1,
            std::bind(&TeleopNode::posesChanged, this, _1)
        );

        this->declare_parameter("frequency", 10);
        this->get_parameter<int>("frequency", frequency_);

        this->declare_parameter("uri1", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        //this->declare_parameter("uri1", "radio://*/80/2M/00E6E6E6E7?safelink=0&autoping=0");
        std::string uri1;
        this->get_parameter<std::string>("uri1", uri1);
        connection_[0] = std::make_shared<Connection>(uri1);
        std::cout << "Connection 1: " << uri1 << std::endl;         

        //this->declare_parameter("uri2", "radio://*/80/2M/00E6E6E6E6?safelink=0&autoping=0");
        this->declare_parameter("uri2", "radio://*/80/2M/E7C2C2C207?safelink=0&autoping=0");

        //this->declare_parameter("uri2", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri2;
        this->get_parameter<std::string>("uri2", uri2);
        connection_[1] = std::make_shared<Connection>(uri2);    

        //this->declare_parameter("uri3", "radio://*/80/2M/00E6E6E6E7?safelink=0&autoping=0");
        this->declare_parameter("uri3", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri3;
        this->get_parameter<std::string>("uri3", uri3);
        connection_[2] = std::make_shared<Connection>(uri3);

        //this->declare_parameter("uri4", "radio://*/80/2M/00E6E6E6E7?safelink=0&autoping=0");
        this->declare_parameter("uri4", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
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

    void publish() 
    {
        // for(int i = 0; i < 4; ++i) {
        //     connection_[i]->send(PacketUtils::cmdLegacy(0, 0, twist_[i].angular.z, twist_[i].linear.z));
        // }
        for (int i =0; i < 4; ++i) {
            
                //connection_[i]->send(PacketUtils::cmdLegacy_Pololu_Teleop(twist_[i].linear.z, twist_[i].angular.z));
                //RCLCPP_INFO(logger_, "sending");
                for (const auto& pose : latest_poses_.poses){
                    //prepare the quaternion for compressing
                    float quat[4] = {
                        (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w
                    };

                    uint32_t comp = quatcompress(quat);

                    //split the compressed quaternion uint32_t into 2 uint16_t for packing in a supported variable type
                    uint16_t quat_first = comp & 0xFFFF;
                    uint16_t quat_second = (comp >> 16) & 0xFFFF;
                    //RCLCPP_INFO(logger_, "ID: %d", getName(pose.name));
                    //RCLCPP_INFO(logger_, "pose is x=%.4f, y=%.4f, z=%.4f", (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z);
                    //connection_[i]->send(PacketUtils::cmdLegacy_Pololu_Teleop(twist_[i].linear.z, twist_[i].angular.z));
                    connection_[i]->send(PacketUtils::motionCapture_Pololu_fullstate(getName(pose.name), (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
                        quat_first, quat_second)); 

                }
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

    static inline uint32_t quatcompress(float const q[4])
        {
            // we send the values of the quaternion's smallest 3 elements.
            unsigned i_largest = 0;
            for (unsigned i = 1; i < 4; ++i) {
                if (fabsf(q[i]) > fabsf(q[i_largest])) {
                    i_largest = i;
                }
            }

            // since -q represents the same rotation as q,
            // transform the quaternion so the largest element is positive.
            // this avoids having to send its sign bit.
            unsigned negate = q[i_largest] < 0;

            // 1/sqrt(2) is the largest possible value 
            // of the second-largest element in a unit quaternion.

            // do compression using sign bit and 9-bit precision per element.
            uint32_t comp = i_largest;
            for (unsigned i = 0; i < 4; ++i) {
                if (i != i_largest) {
                    unsigned negbit = (q[i] < 0) ^ negate;
                    unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / (float)M_SQRT1_2) + 0.5f;
                    comp = (comp << 10) | (negbit << 9) | mag;
                }
            }

	        return comp;
        }

    void quatdecompress(uint32_t comp, float q[4])
    {
        float const SMALL_MAX = 1.0 / sqrt(2);
        unsigned const mask = (1 << 9) - 1;

        int const i_largest = comp >> 30;
        float sum_squares = 0;
        for (int i = 3; i >= 0; --i) {
            if (i != i_largest) {
                unsigned mag = comp & mask;
                unsigned negbit = (comp >> 9) & 0x1;
                comp = comp >> 10;
                q[i] = SMALL_MAX * ((float)mag) / mask;
                if (negbit == 1) {
                    q[i] = -q[i];
                }
                sum_squares += q[i] * q[i];
            }
        }
        q[i_largest] = sqrtf(1.0f - sum_squares);
    }
    

    //parse the last two characters from the name string in case those are numbers and convert them to a robot ID
    static uint8_t getName(const std::string& name)
    {
        if (name.length() >= 2){
            std::string last_two = name.substr(name.length() -2); //cut off all letters up to the last two
            if(std::isdigit(last_two[0]) && std::isdigit(last_two[1])){
                return static_cast<uint8_t>(std::stoi(last_two));
            }
            else if(std::isdigit(last_two[1])){
                return static_cast<uint8_t>(std::stoi(last_two.substr(1)));
            }
        }
        
        return 255; //invalid characters

    }

    
    void posesChanged(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg)
    {
        //mocap_data_received_timepoints_.emplace_back(std::chrono::steady_clock::now());
        //overwrite latest poses
        // std::cout << "Received pose" <<msg->poses.size() << "poses: " <<std::endl;

        // for (const auto& pose : msg->poses) {
        //     std::cout   << "name" << pose.name
        //                 << "position: " << pose.pose.position.x << ","
        //                                 << pose.pose.position.y << ","
        //                                 << pose.pose.position.z << ","
        //                                 << std::endl;
        // }
        latest_poses_ = *msg;
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
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr mocap_subscription;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_[4];
    motion_capture_tracking_interfaces::msg::NamedPoseArray latest_poses_;

    rclcpp::Logger logger_;

    
    int frequency_;
    float dt_;

    std::shared_ptr<Connection> connection_[4];
};

//from     crazyswarm2/crazyflie/src/crazyflie_server.cpp logging helper
// class CrazyflieLogger:public Logger
// {
//     public:
//     CrazyflieLogger(rclcpp::Logger logger, const std::string& prefix)
//         : Logger()
//         , logger_(logger)
//         , prefix_(prefix)
//     {
//     }

//     virtual ~CrazyflieLogger() {}

//     virtual void info(const std::string &msg)
//     {
//         RCLCPP_INFO(logger_, "%s %s", prefix_.c_str(), msg.c_str());
//     }

//     virtual void warning(const std::string &msg)
//     {
//         RCLCPP_WARN(logger_, "%s %s", prefix_.c_str(),  msg.c_str());
//     }

//     virtual void error(const std::string &msg)
//     {
//         RCLCPP_ERROR(logger_, "%s %s", prefix_.c_str(), msg.c_str());
//     }
//     private:
//     rclcpp::Logger logger_;
//     std::string prefix_;
// };

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TeleopNode>());
    rclcpp::shutdown();
    return 0;
}

