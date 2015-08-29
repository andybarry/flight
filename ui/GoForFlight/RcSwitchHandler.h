#ifndef RC_SWITCH_HANDLER_H
#define RC_SWITCH_HANDLER_H

#include "StatusHandler.h"
#include "../../LCM/lcmt/rc_switch_action.hpp"

class RcSwitchHandler : public StatusHandler
{
    public:
        RcSwitchHandler() : StatusHandler("RC Dispatch: ") {
            SetTimeoutThreshold(10*60*1000000); // 10 minutes
        }
        ~RcSwitchHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt::rc_switch_action *msg) {

            SetStatus(true, msg->timestamp);
        }

    private:

};

#endif
