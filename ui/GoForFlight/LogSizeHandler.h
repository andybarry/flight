#ifndef LOG_SIZE_HANDLER_HPP
#define LOG_SIZE_HANDLER_HPP

#include "StatusHandler.h"
#include "../../LCM/lcmt_log_size.hpp"

class LogSizeHandler : public StatusHandler
{
    public:
        LogSizeHandler() : StatusHandler("") {
        }
        ~LogSizeHandler() {}

        void SetTimesyncLabel(wxStaticText *lbl_timesync) { lbl_timesync_ = lbl_timesync; }
        void SetLoggingLabel(wxStaticText *lbl_logging) { lbl_logging_ = lbl_logging; }

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt_log_size* msg) {

            if (msg->timestamp > last_timestamp_) {
                status_timesync_ = true;
            } else {
                status_timesync_ = false;
            }


            if (msg->log_size > last_log_size_) {
                status_logging_ = true;
            } else {
                status_logging_ = false;
            }

            status_ = status_logging_ & status_timesync_;


            last_timestamp_ = msg->timestamp;
            last_log_size_ = msg->log_size;

        }

        void UpdateLabel() {
            lbl_timesync_->SetLabel("Timesync: " + GetStatusString(status_timesync_));
            lbl_timesync_->SetForegroundColour(GetColour(status_timesync_));

            lbl_logging_->SetLabel("Logging: " + GetStatusString(status_logging_));
            lbl_logging_->SetForegroundColour(GetColour(status_logging_));
        }

    private:
        int last_log_size_ = -1;
        long last_timestamp_ = -1;

        bool status_timesync_ = false;
        bool status_logging_ = false;

        wxStaticText *lbl_timesync_;
        wxStaticText *lbl_logging_;


};

#endif // LOG_SIZE_HANDLER_HPP
