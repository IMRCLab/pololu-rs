#include <memory>
#include <vector>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

//includes for sending messages using the crazy radio
#include "crazyflieLinkCpp/Connection.h"
#include "PacketUtils.hpp"

//includes for listening to the Motion Capture Tracking interface poses
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"


using namespace bitcraze::crazyflieLinkCpp;
using std::placeholders::_1;

using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode()
        : Node("controller_interface")
        , logger_(this->get_logger())
    {
        //TODO:
        //implement command sending according to jiamings layout and selection for each robot
        //launch file
        //seperate teleop traj following so that the teleop commands are only monitored and sent if teleop is active for the selected robot.
        //multiple joystick support is not wanted. 


        // for (int i = 0; i < 4; ++i) {
        //     subscription_[i] = this->create_subscription<sensor_msgs::msg::Joy>(
        //         "joy" + std::to_string(i), 1, std::bind(&TeleopNode::joyChanged, this, i, _1));
        // }

        subscription_[0] = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy1", 1, std::bind(&TeleopNode::joyChanged1, this, _1));

        //adjust policy
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        //include listener that will send MoCap package about position to the robot
        mocap_subscription = this->create_subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>(
            "poses", 
            qos,
            std::bind(&TeleopNode::posesChanged, this, _1)
        );

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
        std::cout << "Connection 2: " << uri2 << std::endl;

        this->declare_parameter("uri3", "radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");
        std::string uri3;
        this->get_parameter<std::string>("uri3", uri3);
        connection_[2] = std::make_shared<Connection>(uri3);
        std::cout << "Connection 3: " << uri3 << std::endl;

        this->declare_parameter("uri4", "radio://*/80/2M/E7C2C2C207?safelink=0&autoping=0");
        std::string uri4;
        this->get_parameter<std::string>("uri4", uri4);
        connection_[3] = std::make_shared<Connection>(uri4);
        std::cout << "Connection 4: " << uri4 << std::endl;

        


        // declare cmd_rpy params
        //command params for robot joystick control
        declareAxis("joy1.yawrate");
        declareAxis("joy1.thrust");
        //get command params for the interface button contol:
        getAxis("joy1.yawrate", axes_[0].yaw);
        getAxis("joy1.thrust", axes_[0].z);

        //Frage: warum benutzt man declareAxis und getAxis here?

        dt_ = 1.0f/frequency_;
        if (frequency_ > 0) {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/frequency_), std::bind(&TeleopNode::publish, this));
        }

        RCLCPP_INFO(logger_, "Controller Interface started. Robot %d selected.", selected_robot_ + 1);
        printRobotStatus();
        RCLCPP_INFO(logger_, "Use D-Pad Left/Right to select robots");
    }
    
    
private:

    int selected_robot_ = 0;
    bool prev_buttons_[11] = {false};
    bool prev_dpad_left_ = false;
    bool prev_dpad_right_ = false;  // Add missing semicolon

    bool prev_dpad_up_ = false;
    bool prev_dpad_down_ = false;

    int selected_program_[4] = {0, 0, 0, 0}; // Store selected program for each robot

    bool teleop_activated[4] = {false, false, false, false}; //teleop active for robot
    bool robot_running[4] = {false, false, false, false}; 

    //define available programs
    struct Program {
        std::string name; 
        uint8_t command; //number that is sent to the robot as a command to select the program on the robot: a number from 0 to 255
        std::string description; //leave blank for now
    };

    std::vector<Program> available_programs_ = {
        {"teleop", 0, "Manual TELEOPeration mode"},
        {"trajectory following w/ direct duty", 1, "TRAJ FOLLOWING with DIRECT DUTY"},
        {"traj following w/ mocap", 2, "TRAJ FOLLOWING with MOCAP"}
        //add missing functionality here later
    };

    //match robot ID to the last letters in the address string for matching the selected robot to its id on the firmware derived from the hardware address suffix
    //robot IDs parsed from the radio URI parameters (last part of the radio address)
    std::vector<uint8_t> robot_ids_{255,255,255,255};

    static std::string extractRadioIdFromUri(const std::string &uri)
    {
        if (uri.empty()) return "";
        auto last_slash = uri.find_last_of('/');
        std::string seg = (last_slash == std::string::npos) ? uri : uri.substr(last_slash + 1);
        // strip query part starting with '?'
        auto qpos = seg.find('?');
        if (qpos != std::string::npos) seg = seg.substr(0, qpos);
        return seg;
    }

    void initRobotIds()
    {
        std::string uri;
        if (this->get_parameter<std::string>("uri1", uri)) {
            robot_ids_[0] = getName(extractRadioIdFromUri(uri));
        }
        if (this->get_parameter<std::string>("uri2", uri)) {
            robot_ids_[1] = getName(extractRadioIdFromUri(uri));
        }
        if (this->get_parameter<std::string>("uri3", uri)) {
            robot_ids_[2] = getName(extractRadioIdFromUri(uri));
        }
        if (this->get_parameter<std::string>("uri4", uri)) {
            robot_ids_[3] = getName(extractRadioIdFromUri(uri));
        }
    }


    //axis for teleop control
    struct Axis
    { 
        int axis;
        float max;
        float deadband;
    };
    struct
    {
        Axis z;     // speed for the ground robot
        Axis yaw;   // turning angle for the ground robot. 
    } axes_[1]; //only one controller for now

    // Publish Function, sending tele operation command
    void publish() 
    {
        //handle communication tied to the frequency here. 

        // Only send to selected robot and only if teleop is active
        //RCLCPP_INFO(logger_, "selected robot is %d", selected_robot_);
        //RCLCPP_INFO(logger_, "selected program is %s", available_programs_[selected_program_[selected_robot_]].name.c_str());
        //RCLCPP_INFO(logger_, "teleop activated is %d", teleop_activated[selected_robot_]);
        if (teleop_activated[selected_robot_] == 1) {
            teleopCommand(selected_robot_);
        }   
    }

    void teleopCommand(int robot_id){
        connection_[robot_id]->send(PacketUtils::cmdLegacy_Pololu_Teleop(twist_[0].linear.z, twist_[0].angular.z));
        RCLCPP_INFO(logger_, "sending angular and linear twist with %f and %f", twist_[0].linear.z, twist_[0].angular.z);
        RCLCPP_INFO(logger_, "to robot with id %d", robot_id);
    }


    void joyChanged1(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        //some debug logging
        //RCLCPP_INFO(logger_, "Received joystick message with %zu axes and %zu buttons", msg->axes.size(), msg->buttons.size());
        // Debug button states (only log when buttons are pressed)
        // for (size_t i = 0; i < msg->buttons.size() && i < 8; ++i) {
        //     if (msg->buttons[i] == 1) {
        //         RCLCPP_INFO(logger_, "Button %zu pressed!", i);
        //     }
        // }
        
        // Debug D-Pad axes
        //RCLCPP_INFO(logger_, "D-Pad axes - axis 6: %f, axis 7: %f", msg->axes[6], msg->axes[7]);

        //handle joystick movement
        twist_[0].linear.z = getAxis(msg, axes_[0].z);
        twist_[0].angular.z = getAxis(msg, axes_[0].yaw);
        
        //handle button presses
        handleAllButtons(msg);
        
        //handle robot selection
        handleRobotSelection(msg);
    }

    void broadcastPosition(){
        for (int i = 0; i < 4; ++i)
        {
            for (const auto &pose : latest_poses_.poses)
            {
                // prepare the quaternion for compressing
                float quat[4] = {
                    (float)pose.pose.orientation.x, (float)pose.pose.orientation.y, (float)pose.pose.orientation.z, (float)pose.pose.orientation.w};

                uint32_t comp = quatcompress(quat);

                // split the compressed quaternion uint32_t into 2 uint16_t for packing in a supported variable type
                uint16_t quat_first = comp & 0xFFFF;
                uint16_t quat_second = (comp >> 16) & 0xFFFF;
                RCLCPP_INFO(logger_, "ID: %d", getName(pose.name));
                RCLCPP_INFO(logger_, "pose is x=%.4f, y=%.4f, z=%.4f", (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z);
                // connection_[i]->send(PacketUtils::cmdLegacy_Pololu_Teleop(twist_[i].linear.z, twist_[i].angular.z));

                //only broadcast to robots that are currently not in teleoperation mode:
                if (!teleop_activated[i])
                {
                    connection_[i]->send(PacketUtils::motionCapture_Pololu_fullstate(getName(pose.name), (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z,
                                                                                 quat_first, quat_second));
                }
            }
        }
    }

    static inline uint32_t quatcompress(float const q[4])
    {
        // we send the values of the quaternion's smallest 3 elements.
        unsigned i_largest = 0;
        for (unsigned i = 1; i < 4; ++i)
        {
            if (fabsf(q[i]) > fabsf(q[i_largest]))
            {
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
        for (unsigned i = 0; i < 4; ++i)
        {
            if (i != i_largest)
            {
                unsigned negbit = (q[i] < 0) ^ negate;
                unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / (float)M_SQRT1_2) + 0.5f;
                comp = (comp << 10) | (negbit << 9) | mag;
            }
        }

        return comp;
    }

    //mocap related functionality
    void posesChanged(const motion_capture_tracking_interfaces::msg::NamedPoseArray::SharedPtr msg)
    {
        // mocap_data_received_timepoints_.emplace_back(std::chrono::steady_clock::now());
        // overwrite latest poses
        //  std::cout << "Received pose" <<msg->poses.size() << "poses: " <<std::endl;

        // for (const auto& pose : msg->poses) {
        //     std::cout   << "name" << pose.name
        //                 << "position: " << pose.pose.position.x << ","
        //                                 << pose.pose.position.y << ","
        //                                 << pose.pose.position.z << ","
        //                                 << std::endl;
        // }
        latest_poses_ = *msg;
    }

    static uint8_t getName(const std::string &name)
    {
        if (name.length() >= 2)
        {
            std::string last_two = name.substr(name.length() - 2); // cut off all letters up to the last two
            if (std::isdigit(last_two[0]) && std::isdigit(last_two[1]))
            {
                return static_cast<uint8_t>(std::stoi(last_two));
            }
            else if (std::isdigit(last_two[1]))
            {
                return static_cast<uint8_t>(std::stoi(last_two.substr(1)));
            }
        }

        return 255; // invalid characters
    }

    void handleRobotSelection(const sensor_msgs::msg::Joy::SharedPtr &msg)
    {
        if (msg->axes.size() < 8) {
            RCLCPP_WARN_THROTTLE(logger_, *this->get_clock(), 5000, "Controller has insufficient axes (%zu < 8) for D-Pad", msg->axes.size());
            return;
        }

        //get D-Pad values
        float dpad_horizontal = msg->axes[6]; // Left/Right
        float dpad_vertical = msg->axes[7];   // Up/Down

        //convert to boolean states
        bool dpad_left = dpad_horizontal > 0.5;
        bool dpad_right = dpad_horizontal < -0.5;
        bool dpad_up = dpad_vertical > 0.5;
        bool dpad_down = dpad_vertical < -0.5;

        //robot selection (Left/Right) - your existing working code
        if (dpad_left && !prev_dpad_left_) {
            selected_robot_ = (selected_robot_ + 3) % 4;
            RCLCPP_INFO(logger_, "◀ D-Pad Left: Robot %d selected", selected_robot_ + 1);
            printRobotStatus();
        }
        
        if (dpad_right && !prev_dpad_right_) {
            selected_robot_ = (selected_robot_ + 1) % 4;
            RCLCPP_INFO(logger_, "▶ D-Pad Right: Robot %d selected", selected_robot_ + 1);
            printRobotStatus();
        }

        //program selection (Up/Down) - stored for each robot, else same logic as above
        if (dpad_up && !prev_dpad_up_) {
            selected_program_[selected_robot_] = (selected_program_[selected_robot_] + 1) % available_programs_.size();
            RCLCPP_INFO(logger_, "▲ D-Pad Up: Robot %d program: %s", 
                       selected_robot_ + 1, 
                       available_programs_[selected_program_[selected_robot_]].name.c_str());
            printRobotStatus();
        }
        
        if (dpad_down && !prev_dpad_down_) {
            selected_program_[selected_robot_] = (selected_program_[selected_robot_] + available_programs_.size() - 1) % available_programs_.size();
            RCLCPP_INFO(logger_, "▼ D-Pad Down: Robot %d program: %s", 
                       selected_robot_ + 1, 
                       available_programs_[selected_program_[selected_robot_]].name.c_str());
            printRobotStatus();
        }

        //update previous states
        prev_dpad_left_ = dpad_left;
        prev_dpad_right_ = dpad_right;
        prev_dpad_up_ = dpad_up;
        prev_dpad_down_ = dpad_down;
    }

    void handleAllButtons(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons.size() < 8) return;

        //mapping of the buttons:
        // 0: A, 1: B, 2: X, 3: Y, 4: LB, 5: RB, 6: BACK, 7: START, 8: MIDDLE, 9: LS, 10: RS

        //note: in teleop mode you can't stop the robot by pressing B, use X to quit the program and deadzone for stop it. 
        // START button (7) - Start ALL robots
        if (getButton(msg, 7) && !prev_buttons_[7]) {
            RCLCPP_INFO(logger_, "Button START pressed"); //send quit command to go back to orchestrator interface
            RCLCPP_INFO(logger_, "START: Starting ALL robots");
            sendGlobalStart();
        }
        
        // BACK button (6) - Emergency stop ALL robots
        if (getButton(msg, 6) && !prev_buttons_[6]) {
            RCLCPP_INFO(logger_, "Button BACK pressed");
            RCLCPP_WARN(logger_, "STOP: Stopping ALL robots");
            sendGlobalStop();
        }
        
        // A button (0) - Start selected robot
        if (getButton(msg, 0) && !prev_buttons_[0]) {
            RCLCPP_INFO(logger_, "A: Starting robot %d", selected_robot_ + 1);  // Add +1
            sendIndividualStart(selected_robot_);
        }
        
        // B button (1) - Stop selected robot
        if (getButton(msg, 1) && !prev_buttons_[1]) {
            RCLCPP_INFO(logger_, "B: Stopping robot %d", selected_robot_ + 1);  // Add +1
            sendIndividualStop(selected_robot_);}
        
        // X button (2) - Send/Confirm program to selected robot
        if (getButton(msg, 2) && !prev_buttons_[2]) {
            RCLCPP_INFO(logger_, "Button X pressed");
            //if the robot is currently running a program: quit the program 
            if (robot_running[selected_robot_])
            {
                //RCLCPP_WARN(logger_, "Robot %d is currently running a program. Stop it first before sending a new program.", selected_robot_ + 1);
                //send X again to quit the current program
                RCLCPP_INFO(logger_, "Sending QUIT command to Robot %d to go back to Orchestrator interface", selected_robot_ + 1);
                sendIndividualCommand(selected_robot_, 113); //q in asccii is 113
                robot_running[selected_robot_] = false;
                teleop_activated[selected_robot_] = false; //deactivate teleop always when stopping
            }
            else
            {
                RCLCPP_INFO(logger_, "Program '%s' sent to Robot %d", 
                           available_programs_[selected_program_[selected_robot_]].name.c_str(),
                           selected_robot_ + 1);
                sendProgramCommand(selected_robot_, available_programs_[selected_program_[selected_robot_]].command);
                robot_running[selected_robot_] = true;
            }
        }
        
        //update all button states for edge detection
        for (size_t i = 0; i < msg->buttons.size() && i < 11; ++i) {
            prev_buttons_[i] = getButton(msg, i);
            //RCLCPP_INFO(logger_, "Button %zu state: %d", i, prev_buttons_[i]);
        }
    }

    void printRobotStatus()
    {
        std::string status = "Robot Selection: ";
        for (int i = 0; i < 4; ++i) {
            if (i == selected_robot_) {
                status += "[" + std::to_string(i + 1) + "] ";  // Add +1 here
            } else {
                status += " " + std::to_string(i + 1) + "  ";   // Add +1 here
            }
        }
        RCLCPP_INFO(logger_, "%s", status.c_str());
        //print program for selected robot:
        for (int i = 0; i< 4; ++i){
            if (i == selected_robot_){
                RCLCPP_INFO(logger_, "Selected Program for Robot %d: %s", i + 1, available_programs_[selected_program_[i]].name.c_str());
            }
        }

        RCLCPP_INFO(logger_, "Available Programs:");
        for (uint8_t j = 0; j < available_programs_.size(); ++j) {
            if (j == selected_program_[selected_robot_]) {
                RCLCPP_INFO(logger_, "[%d] %s", j, available_programs_[j].name.c_str());
            } else {
                RCLCPP_INFO(logger_, "%d: %s", j, available_programs_[j].name.c_str());
            }
        }

    }

    //handle the sub robot interface
    

    sensor_msgs::msg::Joy::_buttons_type::value_type getButton(const sensor_msgs::msg::Joy::SharedPtr &msg, int button)
    {
        //Xbox Controller button mapping:
        // 0: A, 1: B, 2: X, 3: Y, 4: LB, 5: RB, 6: BACK, 7: START, 8: MIDDLE, 9: LS, 10: RS
        if (button < 0 || (size_t) button >= msg->buttons.size()) {
            return -1; //out of range
        }
        return msg->buttons[button]; //get button state (0: not pressed, 1: pressed)

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

    void sendIndividualCommand(int robot_id, uint8_t command)
    {
        //ID 255 means no specific robot is the receiver of the command. this is handled on the robot side. 
        connection_[robot_id]->send(PacketUtils::cmdTrajectoryControl(robot_ids_[robot_id], command));
        RCLCPP_INFO(logger_, "Sent command %d to robot %d", command, robot_id + 1);
    }

    void sendIndividualStart(int robot_id){
        connection_[robot_id]->send(PacketUtils::cmdTrajectoryControlStartStop(robot_ids_[robot_id], 1));
        RCLCPP_INFO(logger_, "Sent START command to robot %d", robot_id + 1);
        robot_running[robot_id] = true;
    }

    void sendIndividualStop(int robot_id){

        connection_[robot_id]->send(PacketUtils::cmdTrajectoryControlStartStop(robot_ids_[robot_id], 0));
        RCLCPP_INFO(logger_, "Sent STOP command to robot %d", robot_id + 1);
        robot_running[robot_id] = false;
        teleop_activated[robot_id] = false; //deactivate teleop always when stopping the robot
    }


    void sendGlobalCommand(uint8_t command)
    {
        //global command: broadcasting to all addresses -> all robots receive the same command
        for (int i = 0; i < 4; ++i) {
            // Robot ID 255 means the command is meant for all robots (otherwise the robot ID is sent here for the robot to parse)
            sendIndividualCommand(i, command);
        }
    }

    void sendGlobalStart(){
        for (int i = 0; i < 4; ++i) {
            sendIndividualStart(i);
        }
    }

    void sendGlobalStop(){
        for (int i = 0; i < 4; ++i) {
            sendIndividualStop(i);
        }
    }

    void sendProgramCommand(int robot_id, uint8_t program_command)
    {
        //command sending is done to service Orchestrator task in the pololu-rs firmware.
        //Commands:
        //"T" - teleop mode
        //"D" - trajectory following direct duty
        //"M" - trajectory following mocap

        //from there a start/pause command is expected to start or stop the program (except for the teleop mode, which is active immediately after sending the "T" command)
        //"1" - start (-> A/start)
        //"0" - stop/pause (-> B/back)

        //to go back to the orchestrator interface:
        //"q" - quit current program and go back to orchestrator interface (-> X button)
        switch (program_command) {
            case 0: //teleop
                //send a "T" once
                RCLCPP_INFO(logger_, "Letting robot %d go into TELEOP mode", robot_id + 1);
                sendIndividualCommand(robot_id, 84); //go into teleop mode on the robot "T" = ASCII 84
                teleop_activated[robot_id] = true; //activate teleop for this robot
                break;
            case 1: //trajectory following: direct duty
                RCLCPP_INFO(logger_, "Letting robot  %d go into TRAJECTORY FOLLOWING mode: DIRECT DUTY", robot_id + 1);
                sendIndividualCommand(robot_id, 68); //go into traj following mode selection on the robot "D" = ASCII 68
                //waiting for start command now
                break;
            case 2: //trajectory following: mocap
                RCLCPP_INFO(logger_, "Letting robot  %d go into TRAJECTORY FOLLOWING mode: MOCAP", robot_id + 1);
                sendIndividualCommand(robot_id, 77); //go into traj following mode selection on the robot "M" = ASCII 77
                //waiting for start command now
                break;
            case 3: //Traj Following from demo trajectory
                // add here to expand functionality
                RCLCPP_INFO(logger_, "not implemented!");
                break;
            case 4: //gain tuning mode on demo trajectory
                // add here to expand functionality
                RCLCPP_INFO(logger_, "not implemented!");
                break;
            default:
                RCLCPP_WARN(logger_, "Unknown program command %d for robot %d", program_command, robot_id + 1);
                break;
        }
    }

    void printControlMapping()
    {
        RCLCPP_INFO(logger_, "Xbox Controller Mapping:");
        RCLCPP_INFO(logger_, "  X button - Send/Confirm program to selected robot");
        RCLCPP_INFO(logger_, "  START button - Start ALL robots");
        RCLCPP_INFO(logger_, "  BACK button - Emergency stop ALL robots");
        RCLCPP_INFO(logger_, "  A button - Start selected robot");
        RCLCPP_INFO(logger_, "  B button - Stop selected robot");
        RCLCPP_INFO(logger_, "  D-Pad Left/Right - Select robot (1-4)");
        RCLCPP_INFO(logger_, "  Left Stick - Control selected robot");
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_[1];
    rclcpp::Subscription<motion_capture_tracking_interfaces::msg::NamedPoseArray>::SharedPtr mocap_subscription;
    motion_capture_tracking_interfaces::msg::NamedPoseArray latest_poses_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_[1];
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

