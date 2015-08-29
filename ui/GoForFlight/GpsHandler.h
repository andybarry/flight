#ifndef GPS_HANDLER_H
#define GPS_HANDLER_H

#include "StatusHandler.h"
#include "../../LCM/mav/gps_data_t.hpp"

class GpsHandler : public StatusHandler
{
    public:
        GpsHandler() : StatusHandler("GPS: ") {
        }
        ~GpsHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const mav::gps_data_t* msg) {

            boost::format formatter = boost::format(" (Sats: %d)") % (msg->numSatellites);

            if (msg->gps_lock >= 3) {
                SetStatus(true, msg->utime);
                SetOnlineString(formatter.str());
            } else {
                SetStatus(false, msg->utime);
                SetOfflineString(formatter.str());
            }
        }

    private:

};

#endif // LOG_SIZE_HANDLER_HPP

