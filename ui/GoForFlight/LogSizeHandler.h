#ifndef LOG_SIZE_HANDLER_HPP
#define LOG_SIZE_HANDLER_HPP

#include "MultiStatusHandler.h"
#include "../../LCM/lcmt_log_size.hpp"
#include <boost/format.hpp>

#define MIN_TIMESTAMP_OFFSET_USEC 1000000

#define MIN_DISK_FREE_MB 5000

class LogSizeHandler : public MultiStatusHandler
{
    public:
        LogSizeHandler() : MultiStatusHandler("Logging: ") {

            last_log_number_.resize(3);
            last_log_size_.resize(3);
            last_timestamp_.resize(3);

            for (int i = 0; i < 3; i++) {
                last_log_size_[i] = -1;
                last_log_number_[i] = -1;
            }

            timesync_handler_.SetPrependString("Timesync: ");

            disk_free_handler_.SetPrependString("Disk Free: ");
            disk_free_handler_.SetOnlineString("OK");
            disk_free_handler_.SetOfflineString("Error");
        }

        ~LogSizeHandler() {}


        void SetTimesyncLabelMain(wxStaticText *lbl_timesync) {
            timesync_handler_.SetLabel(lbl_timesync);
        }
        void SetTimesyncLabel(ComputerType index, wxStaticText *lbl_label, wxStaticText *lbl_value) {
            timesync_handler_.AddLabel(index, lbl_label, lbl_value);
        }


        void SetDiskFreeLabelMain(wxStaticText *lbl_disk_free) {
            disk_free_handler_.SetLabel(lbl_disk_free);
        }

        void SetDiskFreeLabel(ComputerType index, wxStaticText *lbl_label, wxStaticText *lbl_value) {
            disk_free_handler_.AddLabel(index, lbl_label, lbl_value);
        }


        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_log_size* msg) {

            ComputerType index = GetIndexFromChannelName(chan);


            if (msg->log_size > last_log_size_[index] && last_log_size_[index] != -1) {

                SetStatus(index, true, msg->timestamp);

                SetText(index, "#" + std::to_string(msg->log_number));

            } else {
                SetStatus(index, false, msg->timestamp);

                if (last_log_number_[index] == -1) {
                    SetText(index, "#--");
                } else {
                    SetText(index, "Static (#" + std::to_string(last_log_number_[index]) + ")");
                }
            }

            bool timestamp_ok = false;
            if (abs(msg->timestamp - StatusHandler::GetTimestampNow()) < MIN_TIMESTAMP_OFFSET_USEC) {
                timestamp_ok = true;
            }

            timesync_handler_.SetStatus(index, timestamp_ok, msg->timestamp);
            timesync_handler_.SetText(index, ParseTime(msg->timestamp));

            bool disk_ok;
            std::string disk_str;

            boost::format formatter = boost::format("%.1f GB") % (msg->disk_space_free / 1024.0);
            disk_str = formatter.str();

            if (msg->disk_space_free < MIN_DISK_FREE_MB) {
                disk_ok = false;

            } else {
                disk_ok = true;
            }

            disk_free_handler_.SetStatus(index, disk_ok, msg->timestamp);
            disk_free_handler_.SetText(index, disk_str);

            last_timestamp_[index] = msg->timestamp;
            last_log_size_[index] = msg->log_size;
            last_log_number_[index] = msg->log_number;

            Update();

        }

        void Update() {
            MultiStatusHandler::Update();

            timesync_handler_.Update();

            disk_free_handler_.Update();
        }

    private:

        std::string ParseTime(long timestamp) {

            char tmbuf[64], buf[64];

            // figure out what time the plane thinks it is
            struct tm *nowtm;
            time_t tv_sec = timestamp / 1000000.0;
            nowtm = localtime(&tv_sec);
            strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d | %H:%M:%S", nowtm);
            sprintf(buf, "%s", tmbuf);

            return std::string(buf);
        }

        std::vector<int> last_log_number_;
        std::vector<long> last_log_size_;

        std::vector<long> last_timestamp_;

        MultiStatusHandler timesync_handler_;
        MultiStatusHandler disk_free_handler_;

};

#endif
