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
        if (selected_program_[selected_robot_] == 0 && teleop_activated[selected_robot_] == 1) {
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

        // START button (7) - Start ALL robots
        if (getButton(msg, 7) && !prev_buttons_[7]) {
            RCLCPP_INFO(logger_, "Button START pressed");
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
            RCLCPP_INFO(logger_, "X: Sending program '%s' to Robot %d", 
                       available_programs_[selected_program_[selected_robot_]].name.c_str(),
                       selected_robot_ + 1);
            //if the robot is currently running a program: forbid the selection
            if (robot_running[selected_robot_])
            {
                RCLCPP_WARN(logger_, "Robot %d is currently running a program. Stop it first before sending a new program.", selected_robot_ + 1);
            }
            else
            {
                RCLCPP_INFO(logger_, "Program '%s' sent to Robot %d", 
                           available_programs_[selected_program_[selected_robot_]].name.c_str(),
                           selected_robot_ + 1);
                sendProgramCommand(selected_robot_, available_programs_[selected_program_[selected_robot_]].command);
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
        connection_[robot_id]->send(PacketUtils::cmdTrajectoryControl(255, command));
        RCLCPP_INFO(logger_, "Sent command %d to robot %d", command, robot_id + 1);
    }

    void sendIndividualStart(int robot_id){
        connection_[robot_id]->send(PacketUtils::cmdTrajectoryControl(255, 1));
        RCLCPP_INFO(logger_, "Sent START command to robot %d", robot_id + 1);
        robot_running[robot_id] = true;
    }

    void sendIndividualStop(int robot_id){
        connection_[robot_id]->send(PacketUtils::cmdTrajectoryControl(255, 0));
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
        //command sending is done in a cascade to respect firmware downwards compatibility:
        switch (program_command) {
            case 0: //teleop
                //send a 0 once
                RCLCPP_INFO(logger_, "Letting robot %d go into TELEOP mode", robot_id + 1);
                //send start command right after to the same robot
                sendIndividualCommand(robot_id, 0); //1 means start command on the robot
                sendIndividualStart(robot_id); //immediately starting teleop 
                teleop_activated[robot_id] = true; //activate teleop for this robot
                break;
            case 1: //trajectory following: direct duty
                RCLCPP_INFO(logger_, "Letting robot  %d go into TRAJECTORY FOLLOWING mode: DIRECT DUTY", robot_id + 1);
                sendIndividualCommand(robot_id, 1); //go into traj following selection on the robot
                sendIndividualCommand(robot_id, 0); //go into direct duty mode traj following
                //waiting for start command now
                break;
            case 2: //trajectory following: mocap
                RCLCPP_INFO(logger_, "Letting robot  %d go into TRAJECTORY FOLLOWING mode: MOCAP", robot_id + 1);
                sendIndividualCommand(robot_id, 1); //go tino traj following mode selection on the robot
                sendIndividualCommand(robot_id, 1); //go into mocap mode traj following
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
        RCLCPP_INFO(logger_, "  START button - Start ALL robots");
        RCLCPP_INFO(logger_, "  BACK button - Emergency stop ALL robots");
        RCLCPP_INFO(logger_, "  A button - Start selected robot");
        RCLCPP_INFO(logger_, "  B button - Stop selected robot");
        RCLCPP_INFO(logger_, "  D-Pad Left/Right - Select robot (1-4)");
        RCLCPP_INFO(logger_, "  Left Stick - Control selected robot");
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_[1];
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

