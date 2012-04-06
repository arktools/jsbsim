#include "arkcomm/AsyncSerial.hpp"
#include <iostream>
#include <stdexcept>
#include <boost/timer.hpp>
#include "FGMAVLink.h" 

FGMAVLink::FGMAVLink(const std::string & device, uint32_t baudRate) : _count(0), _packet_drops(0), _chan(MAVLINK_COMM_0), _clock() {
    if (mavlink_comm_0_port == NULL)
    {
        try
        {
            mavlink_comm_0_port = new BufferedAsyncSerial(device,baudRate);
        }
        catch(const boost::system::system_error & e)
        {
            std::cout << "error: " << e.what() << std::endl;
            exit(1);
        }
    }
}

FGMAVLink::~FGMAVLink() {
    if (mavlink_comm_0_port)
    {
        delete mavlink_comm_0_port;
        mavlink_comm_0_port = NULL;
    }
}

void FGMAVLink::send() {
    // attitude states (rad)
    float roll = 1;
    float pitch = 2;
    float yaw = 3;

    // body rates
    float rollRate = 0.1;
    float pitchRate = 0.2;
    float yawRate = 0.3;

    // position
    int32_t lat = 1*_rad2deg*1e7;
    int32_t lon = 2*_rad2deg*1e7;
    int16_t alt = 3*1e3;

    int16_t vx = 1*1e2;
    int16_t vy = 2*1e2;
    int16_t vz = -3*1e2;

    int16_t xacc = 1*1e3;
    int16_t yacc = 2*1e3;
    int16_t zacc = 3*1e3;

    mavlink_msg_hil_state_send(_chan,_clock.elapsed(),
                               roll,pitch,yaw,
                               rollRate,pitchRate,yawRate,
                               lat,lon,alt,
                               vx,vy,vz,
                               xacc,yacc,zacc);
}

void FGMAVLink::receive() {
   // receive messages
    mavlink_message_t msg;
    mavlink_status_t status;

    while(comm_get_available(MAVLINK_COMM_0))
    {
        uint8_t c = comm_receive_ch(MAVLINK_COMM_0);

        // try to get new message
        if(mavlink_parse_char(MAVLINK_COMM_0,c,&msg,&status))
        {
            switch(msg.msgid)
            {

            case MAVLINK_MSG_ID_HIL_CONTROLS:
            {
                //std::cout << "receiving messages" << std::endl;
                mavlink_hil_controls_t hil_controls;
                mavlink_msg_hil_controls_decode(&msg,&hil_controls);
                std::cout << "roll: " << hil_controls.roll_ailerons << std::endl;
                std::cout << "pitch: " << hil_controls.pitch_elevator << std::endl;
                std::cout << "yaw: " << hil_controls.yaw_rudder << std::endl;
                std::cout << "throttle: " << hil_controls.throttle << std::endl;
                std::cout << "mode: " << hil_controls.mode << std::endl;
                std::cout << "nav mode: " << hil_controls.nav_mode << std::endl;
                break;
            }

            default:
            {
                std::cout << "received message: " << uint32_t(msg.msgid) << std::endl;
            }

            }
        }

        // update packet drop counter
        _packet_drops += status.packet_rx_drop_count;
    }
}

// vim:ts=4:sw=4:expandtab
