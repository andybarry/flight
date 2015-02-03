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

        void SetTimesyncLabels(wxStaticText *lbl_timesync, wxStaticText *lbl_timestamp) {
            lbl_timesync_ = lbl_timesync;
            lbl_timestamp_ = lbl_timestamp;
        }

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
                last_log_ = msg->log_number;
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

            lbl_timestamp_->SetLabel(GetPlaneTime());
            lbl_timestamp_->SetForegroundColour(GetColour(status_timesync_));

            if (last_log_ < 0) {
                lbl_logging_->SetLabel(wxString::Format("Logging: %s (#--)", GetStatusString(status_logging_)));
            } else {
                lbl_logging_->SetLabel(wxString::Format("Logging: %s (#%i)", GetStatusString(status_logging_), last_log_));
            }
            lbl_logging_->SetForegroundColour(GetColour(status_logging_));
        }

    private:

        std::string GetPlaneTime() {
            if (status_timesync_ == false) {
                return "--:--:--";
            }

            char tmbuf[64], buf[64];

            // figure out what time the plane thinks it is
            struct tm *nowtm;
            time_t tv_sec = last_timestamp_ / 1000000.0;
            nowtm = localtime(&tv_sec);
            strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
            sprintf(buf, "%s", tmbuf);

            return std::string(buf);
        }

        int last_log_size_ = -1;
        long last_timestamp_ = -1;

        int last_log_ = -1;

        bool status_timesync_ = false;
        bool status_logging_ = false;

        wxStaticText *lbl_timesync_;
        wxStaticText *lbl_timestamp_;
        wxStaticText *lbl_logging_;


};

#endif // LOG_SIZE_HANDLER_HPP
