#ifndef AIRSPEED_HANDLER_H
#define AIRSPEED_HANDLER_H

#include "../../../pronto-distro/build/include/lcmtypes/mav/indexed_measurement_t.hpp"

#include <deque>

#define MAX_EXPECTED_GROUND_MPS 5
#define MIN_EXPECTED_CHANGE 0.5
#define MAX_NUM_IN_MEAN 100

class AirspeedHandler : public StatusHandler
{
    public:
        AirspeedHandler() : StatusHandler("Airspeed: ") {
            SetOfflineString("Offline (Mean: -- / Std Dev: --)");
            total_measurements_ = 0;

        }
        ~AirspeedHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const mav::indexed_measurement_t* msg) {


            // for airspeed, we expect some noise on the order of 0-4 m/s
            // if we see something over that value, things are bad.
            // if we see something consistently under that value, things are bad.
            // if we see something that isn't moving at all, things are bad

            // compute a running mean of the speed, and of the standard deviation

            speed_values_.push_back(msg->z_effective[0]);

            if (speed_values_.size() > MAX_NUM_IN_MEAN) {
                speed_values_.pop_front();
            }

            // compute statistics
            std::deque<double>::iterator iter;

            double sum = 0;

            for (iter = speed_values_.begin(); iter != speed_values_.end(); iter++) {
                sum += *iter;
            }

            double mean = sum/speed_values_.size();

            double std_dev = 0;

            for (iter = speed_values_.begin(); iter != speed_values_.end(); iter++) {
                std_dev += (*iter - mean) * (*iter - mean);
            }

            std_dev = sqrt ( std_dev / speed_values_.size() );

            boost::format formatter = boost::format("(Mean: %.1f / Std Dev: %.1f)") % mean % std_dev;

            if (mean > MAX_EXPECTED_GROUND_MPS || std_dev < MIN_EXPECTED_CHANGE) {
                SetStatus(false, msg->utime);
                SetOfflineString("Offline " + formatter.str());
            } else {
                SetStatus(true, msg->utime);
                SetOnlineString("Online " + formatter.str());
            }

        }

    private:

        std::deque<double> speed_values_;
        int total_measurements_;

        double rolling_mean_;
        double rolling_stand_div_;





};

#endif

