#ifndef STATE_MACHINE_HANDLER_H
#define STATE_MACHINE_HANDLER_H

#include "StatusHandler.h"
#include "../../LCM/lcmt/debug.hpp"

class StateMachineHandler : public StatusHandler
{
    public:
        StateMachineHandler() : StatusHandler("State Machine: ") {
            SetTimeoutThreshold(10*60*1000000); // 10 minutes
        }
        ~StateMachineHandler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt::debug *msg) {

            // if we get a message, it means the state machine controller is running
            SetStatus(true, msg->utime);
            SetOnlineString(msg->debug);


        }

    private:

};

#endif
