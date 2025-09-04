#include <iostream>
#include <thread>

#include "crazyflieLinkCpp/Connection.h"
#include "PacketUtils.hpp"

using namespace bitcraze::crazyflieLinkCpp;

// int main()
// {
//     Connection connection("radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");


//     for (int i = 0; i < 10; ++i) {
//         std::cout << "Sending cmdLegacy" << std::endl;
//         connection.send(PacketUtils::cmdLegacy(0, 0, 10, 10));
//         std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//     }
   
//     connection.close();
//     std::cout << "Done." << std::endl;
//     return 0;
// }


int main()
{
    Connection connection("radio://*/80/2M/E7C2C2C208?safelink=0&autoping=0");    const float dt = 0.1f; // seconds
    const int steps_per_segment = 500; // each segment runs 50 * 0.1s = 5s
    const float v_forward = 0.50f;
    // const float omega_turn = 1.0f;    // === 1. 直线前进 ===
    for (int i = 0; i < steps_per_segment; ++i) {
        float t = i * dt;
        std::cout << "[Straight1] t=" << t << "s: v=" << v_forward << ", omega=0" << std::endl;
        connection.send(PacketUtils::cmdLegacy_Pololu_Mix(v_forward, 0.0f, 0, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    /*
    // === 2. 左转半圆 ===
    for (int i = 0; i < steps_per_segment; ++i) {
        float t = i * dt;
        std::cout << "[LeftTurn] t=" << t << "s: v=" << v_forward << ", omega=" << omega_turn << std::endl;
        connection.send(PacketUtils::cmdLegacy_Pololu_Mix(v_forward, omega_turn, 0, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    // === 3. 直线前进 ===
    for (int i = 0; i < steps_per_segment; ++i) {
        float t = i * dt;
        std::cout << "[Straight2] t=" << t << "s: v=" << v_forward << ", omega=0" << std::endl;
        connection.send(PacketUtils::cmdLegacy_Pololu_Mix(v_forward, 0.0f, 0, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }    // === 4. 右转半圆 ===
    for (int i = 0; i < steps_per_segment; ++i) {
        float t = i * dt;
        std::cout << "[RightTurn] t=" << t << "s: v=" << v_forward << ", omega=" << -omega_turn << std::endl;
        connection.send(PacketUtils::cmdLegacy_Pololu_Mix(v_forward, -omega_turn, 0, 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    */
    connection.close();
    std::cout << "Track done." << std::endl;
    return 0;
}