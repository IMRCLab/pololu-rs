#include <iostream>
#include <thread>

#include "crazyflieLinkCpp/Connection.h"
#include "PacketUtils.hpp"

using namespace bitcraze::crazyflieLinkCpp;

int main()
{
    /* Creates a connection that will broadcast to all
     * crazyflies on channel 80. Note that this requires
     * updating the Crazyradio firmware from its factory version
     */
    Connection broadcastConnection("radiobroadcast://*/80/2M");


    for (int i = 0; i < 10; ++i) {
        std::cout << "Sending startTrajectory" << std::endl;
        broadcastConnection.send(PacketUtils::startTrajectoryCommand(false, false, 0, 1.0f));
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

/*
    // Requires high level commander enabled (param commander.enHighLevel == 1)
    std::cout << "Taking off..." << std::endl;
    broadcastConnection.send(PacketUtils::takeoffCommand(0.5f, 0.0f, 3.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(3250));

    std::cout << "Landing..." << std::endl;
    broadcastConnection.send(PacketUtils::landCommand(0.0f, 0.0f, 3.0f));
    std::this_thread::sleep_for(std::chrono::milliseconds(3250));

    std::cout << "Stopping..." << std::endl;
    broadcastConnection.send(PacketUtils::stopCommand());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
*/
    
    broadcastConnection.close();
    std::cout << "Done." << std::endl;
    return 0;
}