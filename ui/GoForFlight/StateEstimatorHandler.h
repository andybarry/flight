#ifndef STATE_ESTIMATOR_HANDLER_HPP
#define STATE_ESTIMATOR_HANDLER_HPP

#include "StatusHandler.h"
#include "../../LCM/mav/pose_t.hpp"

class StateEsimatorHandler : public StatusHandler
{
    public:
        StateEsimatorHandler() : StatusHandler("State Estimator: ") {
        }
        ~StateEsimatorHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const mav::pose_t* msg) {

            if (msg->utime > last_utime_) {
                SetStatus(true, msg->utime);
            } else {
                SetStatus(false, msg->utime);
            }

            last_utime_ = msg->utime;

        }

    private:



        long last_utime_ = -1;



};

#endif
