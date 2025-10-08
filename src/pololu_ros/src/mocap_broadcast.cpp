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

//policy compatibility
#include <rclcpp/rclcpp.hpp>

using namespace bitcraze::crazyflieLinkCpp;


using std::placeholders::_1;

using namespace std::chrono_literals;

class MocapBroadcastNode : public rclcpp::Node
{
public:
    MocapBroadcastNode()
        : Node("mocap_broadcast")
        , logger_(this->get_logger())
    {

        //adjust policy
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        //include listener that will send MoCap package about position to the robot
        mocap_subscription = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
            "poses", 
            qos,
            std::bind(&MocapBroadcastNode::posesChanged, this, _1)
        );

        this->declare_parameter("frequency", 10);
        this->get_parameter<int>("frequency", frequency_);

        this->declare_parameter("uri1", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri1;
        this->get_parameter<std::string>("uri1", uri1);
        connection_[0] = std::make_shared<Connection>(uri1);      

        this->declare_parameter("uri2", "radio://*/80/2M/E7C2C2C207?safelink=0&autoping=0");
        std::string uri2;
        this->get_parameter<std::string>("uri2", uri2);
        connection_[1] = std::make_shared<Connection>(uri2);    

        this->declare_parameter("uri3", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri3;
        this->get_parameter<std::string>("uri3", uri3);
        connection_[2] = std::make_shared<Connection>(uri3);

        this->declare_parameter("uri4", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri4;
        this->get_parameter<std::string>("uri4", uri4);
        connection_[3] = std::make_shared<Connection>(uri4);

        dt_ = 1.0f/frequency_;
        if (frequency_ > 0) {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency_), std::bind(&MocapBroadcastNode::publish, this));
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
        for (int i =0; i < 4; ++i) {
            for (const auto& pose : latest_poses_.poses){
                //prepare the quaternion for compressing
                float quat[4] = {
                    (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w
                };

                uint32_t comp = quatcompress(quat);

                //split the compressed quaternion uint32_t into 2 uint16_t for packing in a supported variable type
                uint16_t quat_first = comp & 0xFFFF;
                uint16_t quat_second = (comp >> 16) & 0xFFFF;
                RCLCPP_INFO(logger_, "ID: %d", getName(pose.name));
                RCLCPP_INFO(logger_, "pose is x=%.4f, y=%.4f, z=%.4f", (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z);
                //connection_[i]->send(PacketUtils::cmdLegacy_Pololu_Teleop(twist_[i].linear.z, twist_[i].angular.z));
                connection_[i]->send(PacketUtils::motionCapture_Pololu_fullstate(getName(pose.name), (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
                    quat_first, quat_second)); 

            }
        }
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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MocapBroadcastNode>());
    rclcpp::shutdown();
    return 0;
}

