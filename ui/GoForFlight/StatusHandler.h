#ifndef STATUS_HANDLER_HPP
#define STATUS_HANDLER_HPP

#include <wx/settings.h>
#include <string.h>

class StatusHandler
{
    public:
        StatusHandler(std::string prepend_str) {
            prepend_str_ = prepend_str;
        }
        ~StatusHandler() {}

        void SetLabel(wxStaticText *lbl_to_update) {
            lbl_to_update_ = lbl_to_update;
        }

        bool GetStatus() { return status_; }
        std::string GetStatusString(bool status_in) {
            if (status_in) {
                return prepend_str_ + "Online";
            } else {
                return prepend_str_ + "Offline";
            }
        }

        void UpdateLabel() {
            lbl_to_update_->SetLabel(GetStatusString(status_));

            lbl_to_update_->SetForegroundColour(GetColour(status_));

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

        wxStaticText *lbl_to_update_;





};

#endif // LOG_SIZE_HANDLER_HPP
