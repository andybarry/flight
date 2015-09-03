#ifndef SPACIAL_STEREO_FILTER_HPP
#define SPACIAL_STEREO_FILTER_HPP


#include <iostream>
#include <math.h>
#include <mutex>

#include <lcm/lcm-cpp.hpp>
#include "gtest/gtest.h"

#include "../../LCM/lcmt/stereo.hpp"
#include "../../utils/utils/RealtimeUtils.hpp"


class SpacialStereoFilter {


    public:
        SpacialStereoFilter(float distance_threshold, int number_of_nearby_points_threshold);
        const lcmt::stereo* ProcessMessage(const lcmt::stereo &msg);

    private:
        static float DistanceFunction(float x1, float x2, float y1, float y2, float z1, float z2);

        int num_points_threshold_;
        float distance_threshold_;

};

#endif
