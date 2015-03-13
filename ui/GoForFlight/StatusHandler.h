#ifndef STATUS_HANDLER_HPP
#define STATUS_HANDLER_HPP

#include <wx/settings.h>
#include <string.h>
#include <sys/time.h>

#define MAX_MESSAGE_DELAY_USEC 2000000

class StatusHandler
{
    public:
        StatusHandler(std::string prepend_str) {
            prepend_str_ = prepend_str;
            online_str_ = "Online";
            offline_str_ = "Offline";
            timeout_ = false;
        }

        ~StatusHandler() {}

        void SetOnlineString(std::string str) {
            online_str_ = str;
        }

        void SetOfflineString(std::string str) {
            offline_str_ = str;
        }

        void SetPrependString(std::string str) {
            prepend_str_ = str;
        }

        std::string GetPrependString() {
            return prepend_str_;
        }


        void SetLabel(wxStaticText *lbl_to_update) {
            lbl_to_update_ = lbl_to_update;
        }

        void SetText(std::string text) {
            if (lbl_to_update_ != NULL) {
                lbl_to_update_->SetLabel(text);
            }
        }

        void SetColour(wxColor color) {
            if (lbl_to_update_ != NULL) {
                lbl_to_update_->SetForegroundColour(color);
            }
        }

        bool GetStatus() { return status_; }

        void SetStatus(bool status, long utime) {
            status_ = status;
            utime_ = utime;
        }

        void SetTimeout(bool timeout) {
            timeout_ = timeout;

            if (timeout == true) {
                SetStatus(false, utime_);
            }
        }

        std::string GetStatusString(bool status_in) {

            if (status_in) {
                return prepend_str_ + online_str_;
            } else {
                return prepend_str_ + offline_str_;
            }
        }

        std::string GetTimeoutString() {
            if (timeout_ == true) {
                return " (timeout)";
            } else {
                return "";
            }
        }

        long GetLastUtime() { return utime_; }

        void CheckTime() {
            if (abs(utime_ - StatusHandler::GetTimestampNow()) > MAX_MESSAGE_DELAY_USEC) {
                if (GetStatus() == true) {
                    SetTimeout(true);
                }
            } else {
                SetTimeout(false);
            }
        }

        void Update() {
            CheckTime();

            SetText(GetStatusString(GetStatus()) + GetTimeoutString());
            SetColour(GetColour(GetStatus()));
        }

        static wxColor GetColour(bool status_in) {
            if (status_in) {
                return wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT);
            } else {
                return wxColor(255, 0, 0);
            }
        }

        static int64_t GetTimestampNow() {
            struct timeval thisTime;
            gettimeofday(&thisTime, NULL);
            return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
        }

    private:

        bool status_;
        std::string prepend_str_;

        std::string online_str_;
        std::string offline_str_;

        long utime_ = -1;

        wxStaticText *lbl_to_update_;

        bool timeout_;

};

#endif // LOG_SIZE_HANDLER_HPP
