#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
// mavlink system definition and headers
#include "arkcomm/asio_mavlink_bridge.h"
#include <mavlink/v1.0/common/mavlink.h>

class FGMAVLink {
public:
    FGMAVLink(const std::string & device, uint32_t baudRate);
    ~FGMAVLink();
    void send();
    void receive();
private:
    uint16_t _count;
    uint16_t _packet_drops;
    mavlink_channel_t _chan;
    boost::timer _clock;
    static const double _rad2deg = 180.0/3.14159;
};

// vim:ts=4:sw=4:expandtab
