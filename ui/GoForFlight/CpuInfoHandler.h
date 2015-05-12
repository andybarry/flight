#ifndef CPU_INFO_HANDLER_H
#define CPU_INFO_HANDLER_H

#define MAX_CPU_C_ODROID 81
#define CPU_SPEED_ODROID 1704000

#define MAX_CPU_C_LOCAL 95

#include "../../LCM/lcmt_cpu_info.hpp"

class CpuInfoHandler : public MultiStatusHandler
{
    public:
        CpuInfoHandler() : MultiStatusHandler("CPU: ") {
            SetOnlineString("OK");
            SetOfflineString("Error");
        }

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_cpu_info* msg) {


            ComputerType index = GetIndexFromChannelName(chan);
            boost::format formatter;

            if (msg->fan_pwm > 0) {
                double percent = msg->fan_pwm / 255.0 * 100.0;
                if (msg->fan_pwm == 1) {
                    percent = 0;
                }
                formatter = boost::format("%.1f Ghz / %.0f C / Fan: %.0f%%") % (msg->cpu_freq / 1000000.0) % msg->cpu_temp % percent;
            } else {
                formatter = boost::format("%.1f Ghz / %.0f C") % (msg->cpu_freq / 1000000.0) % msg->cpu_temp;
            }
            std::string text = formatter.str();

            SetText(index, text);

            bool status;

            if (index == LOCAL) {
                if (msg->cpu_temp >= MAX_CPU_C_LOCAL) {
                    status = false;
                } else {
                    status = true;
                }
            } else {
                if (msg->cpu_temp >= MAX_CPU_C_ODROID || msg->cpu_freq < CPU_SPEED_ODROID) {
                    status = false;
                } else {
                    status = true;
                }
            }


            SetStatus(index, status, msg->timestamp);

        }


};

#endif

