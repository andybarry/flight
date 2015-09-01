#ifndef GIT_STATUS_HANDLER_H
#define GIT_STATUS_HANDLER_H

#include "../../LCM/lcmt/git_status.hpp"

class GitStatusHandler : public MultiStatusHandler
{
    public:
        GitStatusHandler() : MultiStatusHandler("Code sync: ") {
            SetOnlineString("OK");
            SetOfflineString("Error");
            SetTimeoutThreshold(6000000);
        }

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const lcmt::git_status* msg) {

            ComputerType index = GetIndexFromChannelName(chan);

            if (index != LOCAL && sha[0].length() == 0) {
                return;
            }

            sha[index] = msg->sha;
            sha_last_build[index] = msg->sha_last_build;
            last_build_timestamp[index] = msg->last_build_timestamp;

            bool status = true;
            std::string text = "OK";

            if (sha[index].compare(sha[LOCAL]) != 0) {
                status = false;
                text = "SHA mismatch";
            } else {

                if (sha_last_build[index].compare(sha[LOCAL]) != 0) {
                    status = false;
                    text = "Compilation required";
                }
            }

            SetText(index, text);
            SetStatus(index, status, msg->timestamp);

        }

    private:
        std::string sha[NUM_TYPES];
        std::string sha_last_build[NUM_TYPES];
        std::string last_build_timestamp[NUM_TYPES];

};

#endif

