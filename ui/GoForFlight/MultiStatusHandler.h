#ifndef MULTI_STATUS_HANDLER_HPP
#define MULTI_STATUS_HANDLER_HPP

#include <wx/settings.h>
#include <string.h>
#include <vector>
#include <iostream>

#include "StatusHandler.h"

#define NUM_TYPES 3
enum ComputerType { LOCAL = 0, GPS = 1, CAM = 2 };

class MultiStatusHandler : public StatusHandler
{
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
        }

        ~MultiStatusHandler() {}

        void AddLabel(ComputerType type, wxStaticText *label, wxStaticText *value) {
            lbl_labels_[type] = label;
            lbl_values_[type] = value;
        }

        void SetStatus(ComputerType type, bool value) {
            status_array_[type] = value;

            if (lbl_labels_[type] != NULL) {
                lbl_labels_[type]->SetForegroundColour(GetColour(value));
            }

            if (lbl_values_[type] != NULL) {
                lbl_values_[type]->SetForegroundColour(GetColour(value));
            }
        }

        void SetText(ComputerType type, std::string text) {

            if (lbl_values_[type] != NULL) {
                lbl_values_[type]->SetLabel(text);
            }
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
            StatusHandler::Update();


            for (int i = 0; i < NUM_TYPES; i++) {
                if (lbl_labels_[i] != NULL) {
                    lbl_labels_[i]->SetForegroundColour(GetColour(status_array_[i]));
                }

                if (lbl_values_[i] != NULL) {
                    lbl_values_[i]->SetForegroundColour(GetColour(status_array_[i]));
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


};

#endif // LOG_SIZE_HANDLER_HPP

