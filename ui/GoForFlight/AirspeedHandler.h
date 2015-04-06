#ifndef AIRSPEED_HANDLER_H
#define AIRSPEED_HANDLER_H

#include "../../../pronto-distro/build/include/lcmtypes/mav/indexed_measurement_t.hpp"
#include "../../utils/RollingStatistics/RollingStatistics.hpp"

#define MAX_EXPECTED_GROUND_MPS 5
#define MIN_EXPECTED_CHANGE 0.5
#define MAX_NUM_IN_MEAN 100

class AirspeedHandler : public StatusHandler
{
    public:
        AirspeedHandler() : StatusHandler("Airspeed: ") {
            SetOfflineString("Offline (Mean: -- / Std Dev: --)");

            rolling_stats_ = new RollingStatistics(MAX_NUM_IN_MEAN);

        }
        ~AirspeedHandler() {
            delete rolling_stats_;
        }

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const mav::indexed_measurement_t* msg) {


            // for airspeed, we expect some noise on the order of 0-4 m/s
            // if we see something over that value, things are bad.
            // if we see something consistently under that value, things are bad.
            // if we see something that isn't moving at all, things are bad

            // compute a running mean of the speed, and of the standard deviation

            rolling_stats_->AddValue(msg->z_effective[0]);

            double mean = rolling_stats_->GetMean();
            double std_dev = rolling_stats_->GetStandardDeviation();

            boost::format formatter = boost::format("(Mean: %.1f / Std Dev: %.1f)") % mean % std_dev;

            if (mean > MAX_EXPECTED_GROUND_MPS || isnan(std_dev) || std_dev < MIN_EXPECTED_CHANGE) {
                SetStatus(false, msg->utime);
                SetOfflineString("Offline " + formatter.str());
            } else {
                SetStatus(true, msg->utime);
                SetOnlineString("Online " + formatter.str());
            }

        }

    private:

        RollingStatistics *rolling_stats_;





};

#endif

