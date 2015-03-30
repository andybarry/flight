#ifndef ROLLING_STATS_HPP
#define ROLLING_STATS_HPP

#include <deque>
#include <math.h>

#include <iostream>

#include "gtest/gtest.h"


class RollingStatistics {

    public:
        RollingStatistics(unsigned int window_size);

        void AddValue(double new_value);
        double GetMean() { return current_mean_; }
        double GetStandardDeviation() { return sqrt(current_variance_); }


    private:

        double current_mean_;
        double current_variance_;

        unsigned int window_size_;

        std::deque<double> queue_;




};

#endif
