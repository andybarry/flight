#ifndef STEREO_HANDLER_H
#define STEREO_HANDLER_H

#include "StatusHandler.h"
#include "../../LCM/lcmt_stereo_monitor.hpp"
#include <boost/format.hpp>

class StereoHandler : public StatusHandler
{
    public:
        StereoHandler() : StatusHandler("Vision: ") {
        }
        ~StereoHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_stereo_monitor* msg) {


            boost::format formatter = boost::format(" (%02d -- %d)") % msg->video_number % msg->frame_number;

            if (msg->frame_number > 0) {
                SetStatus(true, msg->timestamp);
                SetOnlineString(formatter.str());
                SetOfflineString(formatter.str());

            } else {
                SetStatus(false, msg->timestamp);
                SetOfflineString(formatter.str());
            }

        }

    private:

};

#endif

