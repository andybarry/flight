#ifndef STATUS_HANDLER_HPP
#define STATUS_HANDLER_HPP

#include <wx/settings.h>
#include <string.h>

class StatusHandler
{
    public:
        StatusHandler(std::string prepend_str) {
            prepend_str_ = prepend_str;
            online_str_ = "Online";
            offline_str_ = "Offline";
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

        void SetLabel(wxStaticText *lbl_to_update) {
            lbl_to_update_ = lbl_to_update;
        }

        bool GetStatus() { return status_; }
        std::string GetStatusString(bool status_in) {
            if (status_in) {
                return prepend_str_ + online_str_;
            } else {
                return prepend_str_ + offline_str_;
            }
        }

        void Update() {

            if (lbl_to_update_ != NULL) {
                lbl_to_update_->SetLabel(GetStatusString(status_));

                lbl_to_update_->SetForegroundColour(GetColour(status_));
            }
        }

        static wxColor GetColour(bool status_in) {
            if (status_in) {
                return wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT);
            } else {
                return wxColor(255, 0, 0);
            }
        }

    protected:

        bool status_;
        std::string prepend_str_;

        std::string online_str_;
        std::string offline_str_;

        wxStaticText *lbl_to_update_;





};

#endif // LOG_SIZE_HANDLER_HPP
