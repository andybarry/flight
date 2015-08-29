
#ifndef BATTERY_STATUS_HANDLER_H
#define BATTERY_STATUS_HANDLER_H

#include "StatusHandler.h"
#include "../../LCM/lcmt_battery_status.hpp"
#include <boost/format.hpp>

#define MIN_VOLTAGE 7.2

class BatteryStatusHandler : public StatusHandler
{
    public:
        BatteryStatusHandler() : StatusHandler("Battery Status: ") {
        }
        ~BatteryStatusHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_battery_status* msg) {

            boost::format formatter = boost::format("%.1f V") % (msg->voltage);

            if (msg->voltage > MIN_VOLTAGE) {
                SetStatus(true, msg->timestamp);
                SetOnlineString(formatter.str());

            } else {
                SetStatus(false, msg->timestamp);
                SetOfflineString(formatter.str());
            }
        }

    private:

};

#endif
