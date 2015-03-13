#ifndef DEBUG_HANDLER_HPP
#define DEBUG_HANDLER_HPP

#include "StatusHandler.h"
#include "../../LCM/lcmt_debug.hpp"

#define MAX_MESSAGE_DELAY_USEC_DEBUG 5000000

class DebugHandler : public StatusHandler
{
    public:
        DebugHandler() : StatusHandler("Debug: ") {
            SetOnlineString("OK");

        }
        ~DebugHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_debug* msg) {

            if (msg->utime > GetLastUtime()) {
                SetStatus(false, msg->utime); // getting debug messages means we are NOT ready
                SetOfflineString(msg->debug);
            } else {
                SetStatus(true, msg->utime);
            }

        }

        void Update() {
            CheckTime();

            SetText(GetStatusString(GetStatus()) + GetTimeoutString());
            SetColour(GetColour(GetStatus()));
        }

        void CheckTime() {
            // if we have a timeout, we are OK
            if (abs(GetLastUtime() - StatusHandler::GetTimestampNow()) > MAX_MESSAGE_DELAY_USEC_DEBUG) {
                SetStatus(true, StatusHandler::GetTimestampNow());
            }
        }

    private:





};

#endif

