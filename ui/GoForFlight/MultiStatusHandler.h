#ifndef MULTI_STATUS_HANDLER_HPP
#define MULTI_STATUS_HANDLER_HPP

#include <wx/settings.h>
#include <string.h>
#include <vector>
#include <iostream>

#include "StatusHandler.h"

#define NUM_TYPES 3
enum ComputerType { LOCAL = 0, GPS = 1, CAM = 2 };

class MultiStatusHandler : public StatusHandler {
    public:
        MultiStatusHandler(std::string prepend_str = "") : StatusHandler(prepend_str) {

            lbl_labels_.resize(NUM_TYPES);

            for (int i = 0; i < NUM_TYPES; i++) {
                lbl_labels_[i] = NULL;
            }

            lbl_values_.resize(NUM_TYPES);
            for (int i = 0; i < NUM_TYPES; i++) {
                lbl_values_[i] = NULL;
            }

            status_array_.resize(NUM_TYPES);
            for (int i = 0; i < NUM_TYPES; i++) {
                status_array_[i] = false;
            }

            last_utime_.resize(NUM_TYPES);
            for (int i = 0; i < NUM_TYPES; i++) {
                last_utime_[i] = -1;
            }

            timeout_array_.resize(NUM_TYPES);
            for (int i = 0; i < NUM_TYPES; i++) {
                timeout_array_[i] = false;
            }

            label_text_.resize(NUM_TYPES);
            for (int i = 0; i < NUM_TYPES; i++) {
                label_text_[i] = "";
            }

        }

        ~MultiStatusHandler() {}

        void AddLabel(ComputerType type, wxStaticText *label, wxStaticText *value) {
            lbl_labels_[type] = label;
            lbl_values_[type] = value;
        }

        void SetStatus(ComputerType type, bool value, long utime) {
            status_array_[type] = value;

            if (lbl_labels_[type] != NULL) {
                lbl_labels_[type]->SetForegroundColour(GetColour(value));
            }

            if (lbl_values_[type] != NULL) {
                lbl_values_[type]->SetForegroundColour(GetColour(value));
            }

            last_utime_[type] = utime;
        }

        bool GetStatus(ComputerType type) {
            return status_array_[type];
        }


        void SetText(ComputerType type, std::string text) {

            label_text_[type] = text;

            if (lbl_values_[type] != NULL) {
                lbl_values_[type]->SetLabel(text);
            }
        }

        void SetTimeout(ComputerType type, bool timeout) {
            timeout_array_[type] = timeout;

            if (GetStatus(type) == true && timeout_array_[type]) {

                std::string text = label_text_[type] + " (timeout)";
                SetStatus(type, false, last_utime_[type]);

                if (lbl_values_[type] != NULL) {
                    lbl_values_[type]->SetLabel(text);
                }
            }
        }

        std::string GetText(ComputerType type) {
            std::string str = "";

            if (lbl_values_[type] != NULL) {
                str = std::string(lbl_values_[type]->GetLabel());
            }

            return str;
        }

        bool GetStatus() {
            for (auto status : status_array_) {
                if (status == false) {
                    return false;
                }
            }
            return true;
        }

        void Update() {

            CheckTime();

            StatusHandler::SetText(GetStatusString(GetStatus()));
            SetColour(GetColour(GetStatus()));


            for (int i = 0; i < NUM_TYPES; i++) {
                if (lbl_labels_[i] != NULL) {
                    lbl_labels_[i]->SetForegroundColour(GetColour(status_array_[i]));
                }

                if (lbl_values_[i] != NULL) {
                    lbl_values_[i]->SetForegroundColour(GetColour(status_array_[i]));
                }
            }


        }

        void CheckTime() {

            StatusHandler::CheckTime();

            for (int i = 0; i < NUM_TYPES; i++) {
                if (abs(last_utime_[i] - StatusHandler::GetTimestampNow()) > timeout_threshold_) {
                    SetTimeout(ComputerType(i), true);
                } else {
                    SetTimeout(ComputerType(i), false);
                }
            }


        }

    protected:

        ComputerType GetIndexFromChannelName(std::string channel_name) {
            if (channel_name.find("gps") != std::string::npos) {
                return GPS;

            } else if (channel_name.find("cam") != std::string::npos) {

                return CAM;

            } else {

                return LOCAL;
            }
        }

    private:

        std::vector<wxStaticText*> lbl_labels_;
        std::vector<wxStaticText*> lbl_values_;

        std::vector<bool> status_array_;
        std::vector<long> last_utime_;
        std::vector<bool> timeout_array_;
        std::vector<std::string> label_text_;


};

#endif // LOG_SIZE_HANDLER_HPP

