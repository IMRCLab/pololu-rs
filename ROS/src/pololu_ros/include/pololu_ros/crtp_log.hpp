// CRTP log structs, vendored from W. Hönig's crazyflie_cpp (crtp.h).
#pragma once

#include "crazyflieLinkCpp/Packet.hpp"

namespace pololu_ros {

class crtpLogStartRequest : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogStartRequest(uint8_t id, uint8_t period)
      : Packet(5, 1, 3)
  {
    setPayloadAt<uint8_t>(0, 3);       // START_BLOCK
    setPayloadAt<uint8_t>(1, id);      // block id
    setPayloadAt<uint8_t>(2, period);  // period, x10ms
  }
};

class crtpLogStopRequest : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogStopRequest(uint8_t id)
      : Packet(5, 1, 2)
  {
    setPayloadAt<uint8_t>(0, 4);   // STOP_BLOCK
    setPayloadAt<uint8_t>(1, id);  // block id
  }
};

class crtpLogResetRequest : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogResetRequest()
      : Packet(5, 1, 1)
  {
    setPayloadAt<uint8_t>(0, 5);   // RESET
  }
};

// port 5 ch 2: [blockId:1][ts:3][values...]
struct crtpLogDataResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 5 && p.channel() == 2 && p.payloadSize() > 3;
  }

  static uint8_t blockId(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(0);
  }

  static uint32_t timestampMS(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    uint8_t lo = p.payloadAt<uint8_t>(1);
    uint16_t hi = p.payloadAt<uint16_t>(2);
    return ((uint32_t)hi << 8) | lo;
  }

  template <typename T>
  static T variableAt(const bitcraze::crazyflieLinkCpp::Packet &p, uint8_t idx)
  {
    return p.payloadAt<T>(4 + idx);  // values start at payload byte 4
  }
};

} // namespace pololu_ros
