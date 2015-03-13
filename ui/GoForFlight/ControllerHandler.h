#ifndef CONTROLLER_HANDLER_HPP
#define CONTROLLER_HANDLER_HPP

#include "StatusHandler.h"
#include "../../LCM/lcmt_deltawing_u.hpp"

class ControllerHandler : public StatusHandler
{
    public:
        ControllerHandler() : StatusHandler("Controller: ") {
        }
        ~ControllerHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_deltawing_u* msg) {

            if (msg->timestamp > last_utime_) {
                SetStatus(true, msg->timestamp);
            } else {
                SetStatus(false, msg->timestamp);
            }

            last_utime_ = msg->timestamp;

        }

    private:



        long last_utime_ = -1;



};

#endif
