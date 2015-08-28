#ifndef GIT_MONITOR_HPP
#define GIT_MONITOR_HPP

#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "../../utils/utils/RealtimeUtils.hpp"
#include "../../LCM/lcmt/git_status.hpp"
#include <fstream>

class GitMonitor {

    public:
        GitMonitor(lcm::LCM *lcm, std::string git_status_channel);
        void PublishMessage();


    private:
        lcm::LCM *lcm_;
        std::string git_status_channel_;

};


#endif
