#ifndef STEREO_FILTER_HPP
#define STEREO_FILTER_HPP


#include <iostream>
#include <math.h>
#include <mutex>

#include <lcm/lcm-cpp.hpp>
#include "gtest/gtest.h"

#include "../../LCM/lcmt/stereo.hpp"
#include "../../utils/utils/RealtimeUtils.hpp"


class StereoFilter {


    public:
        StereoFilter(float distance_threshold);
        const lcmt::stereo* ProcessMessage(const lcmt::stereo &msg);

    private:


        bool FilterSinglePoint(float x, float y, float z);
        float DistanceFunction(float x1, float x2, float y1, float y2, float z1, float z2);

        void PrintMsg(const lcmt::stereo &msg, std::string header = "begin message") const;

        const lcmt::stereo *last_stereo_msg_;

        float distance_threshold_;

        std::mutex process_mutex_;


};

#endif
