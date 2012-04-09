#include <arkcomm/AsyncSerial.hpp>
#include <boost/timer.hpp>

// mavlink system definition and headers
#include "mavlink/v1.0/common/mavlink.h"

namespace JSBSim {

class FGMAVLink {

private:

    // private attributes
    
    mavlink_system_t _system;
    mavlink_status_t _status;
    boost::timer _clock;
    BufferedAsyncSerial * _comm;
    static const double _rad2deg = 180.0/3.14159;

    // private methods
    
    // send a mavlink message to the comm port
    void _sendMessage(const mavlink_message_t & msg);

public:
    FGMAVLink(const uint8_t sysid, const uint8_t compid, const MAV_TYPE type,
            const std::string & device, const uint32_t baudRate);
    ~FGMAVLink();
    void send();
    void receive();
};

} // namespace JSBSim

// vim:ts=4:sw=4:expandtab
