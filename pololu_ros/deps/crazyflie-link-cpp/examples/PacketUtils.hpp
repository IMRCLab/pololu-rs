#pragma once
#include "crazyflieLinkCpp/Connection.h"

class PacketUtils
{
public:
    // Constructs a high level commander takeoff packet
    static bitcraze::crazyflieLinkCpp::Packet takeoffCommand(float height, float yaw, float time) {
        const uint8_t size = 16;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)7);    // Command (Takeoff = 7)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        index += pack(buffer.data(), index, height);        // Height
        index += pack(buffer.data(), index, yaw);           // Yaw
        index += pack(buffer.data(), index, true);          // Use Current Yaw
        index += pack(buffer.data(), index, time);          // Duration (seconds)
        
        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x08);          // PORT 8 = HIGH_LEVEL_COMMANDER
        return packet;
    }

    // Constructs a high level commander land packet
    static bitcraze::crazyflieLinkCpp::Packet landCommand(float height, float yaw, float time) {
        const uint8_t size = 16;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)8);    // Command (Land = 8)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        index += pack(buffer.data(), index, height);        // Height
        index += pack(buffer.data(), index, yaw);           // Yaw
        index += pack(buffer.data(), index, true);          // Use Current Yaw
        index += pack(buffer.data(), index, time);          // Duration (seconds)
        
        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x08);          // PORT 8 = HIGH_LEVEL_COMMANDER
        return packet;
    }

    // Constructs a high level commander stop packet
    static bitcraze::crazyflieLinkCpp::Packet stopCommand() {
        const uint8_t size = 3;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)3);    // Command (Stop = 3)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)

        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x08);          // PORT 8 = HIGH_LEVEL_COMMANDER
        return packet;
    }

    // Constructs a high level commander startTrajectory packet
    static bitcraze::crazyflieLinkCpp::Packet startTrajectoryCommand(bool relative, bool reversed, uint8_t trajectoryId, float timescale) {
        const uint8_t size = 10;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, (uint8_t)5);    // Command (Takeoff = 7)
        index += pack(buffer.data(), index, (uint8_t)0);    // Group Mask (0 = all groups)
        index += pack(buffer.data(), index, relative);      // set to true, if trajectory should be shifted to current setpoint
        index += pack(buffer.data(), index, reversed);      // set to true, if trajectory should be executed in reverse
        index += pack(buffer.data(), index, trajectoryId);  // id of the trajectory (previously defined by COMMAND_DEFINE_TRAJECTORY)
        index += pack(buffer.data(), index, timescale);     // time factor; 1 = original speed; >1: slower; <1: faster
        
        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x08);          // PORT 8 = HIGH_LEVEL_COMMANDER
        packet.setChannel((uint8_t) 0x00);
        return packet;
    }

    // Constructs a legacy commnd pcket
    static bitcraze::crazyflieLinkCpp::Packet cmdLegacy(float roll, float pitch, float yawrate, uint16_t thrust) {
        const uint8_t size = 15;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, roll);
        index += pack(buffer.data(), index, pitch);
        index += pack(buffer.data(), index, yawrate);
        index += pack(buffer.data(), index, thrust);
        
        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x03);          // PORT 3
        packet.setChannel((uint8_t) 0x00);
        return packet;
    }

    static bitcraze::crazyflieLinkCpp::Packet cmdLegacy_Pololu(uint16_t left_pwm, uint16_t right_pwm, uint16_t left_dir, uint16_t right_dir) {
        const uint8_t size = 9;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, left_pwm);
        index += pack(buffer.data(), index, right_pwm);
        index += pack(buffer.data(), index, left_dir);
        index += pack(buffer.data(), index, right_dir);        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x03);          // PORT 3
        packet.setChannel((uint8_t) 0x00);
        return packet;
    }    static bitcraze::crazyflieLinkCpp::Packet cmdLegacy_Pololu_Mix(float left_pwm, float right_pwm, float left_dir, float right_dir) {
        const uint8_t size = 17;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, left_pwm);
        index += pack(buffer.data(), index, right_pwm);
        index += pack(buffer.data(), index, left_dir);
        index += pack(buffer.data(), index, right_dir);        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x03);          // PORT 3
        packet.setChannel((uint8_t) 0x00);
        return packet;
    }

    static bitcraze::crazyflieLinkCpp::Packet cmdLegacy_Pololu_Teleop(float linear_velocity, float steering_angle) {
        const uint8_t size = 9;
        std::array<uint8_t, size> buffer;
        buffer[0] = 0xFF;
        uint8_t index = 1;
        index += pack(buffer.data(), index, linear_velocity);
        index += pack(buffer.data(), index, steering_angle);
        bitcraze::crazyflieLinkCpp::Packet packet(buffer.data(), size);
        packet.setPort((uint8_t)0x03);          // PORT 3
        packet.setChannel((uint8_t) 0x00);
        return packet;
    }




private:
    static size_t pack(uint8_t* buffer, uint8_t index, float value) {
        union {
            float value;
            unsigned char bytes[4];
        } converter;
        converter.value = value;
        buffer[index + 0] = converter.bytes[0];
        buffer[index + 1] = converter.bytes[1];
        buffer[index + 2] = converter.bytes[2];
        buffer[index + 3] = converter.bytes[3];
        return 4;
    }

    static size_t pack(uint8_t* buffer, uint8_t index, uint16_t value) {
        union {
            uint16_t value;
            unsigned char bytes[2];
        } converter;
        converter.value = value;
        buffer[index + 0] = converter.bytes[0];
        buffer[index + 1] = converter.bytes[1];
        return 2;
    }

    static size_t pack(uint8_t* buffer, uint8_t index, uint8_t value) {
        buffer[index] = value;
        return 1;
    }

    static size_t pack(uint8_t* buffer, uint8_t index, bool value) {
        uint8_t byteVal = (value) ? 1 : 0;
        buffer[index] = byteVal;
        return 1;
    }
};