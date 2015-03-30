#include "RollingStatistics.hpp"


RollingStatistics::RollingStatistics(unsigned int window_size) {
    window_size_ = window_size;
    current_mean_ = 0;
    current_variance_ = 0;
}


/**
 * Adds a value and computes statistics for that value, since this allows us to compute
 * stats based on the incoming and outgoing value, instead of recomputing every time.
 *
 * See: http://jonisalonen.com/2014/efficient-and-accurate-rolling-standard-deviation/
 *
 */
void RollingStatistics::AddValue(double new_value) {

    queue_.push_back(new_value);

    if (queue_.size() > window_size_) {
        double old_value = queue_.front();
        queue_.pop_front();


        // compute rolling stats
        double old_mean = current_mean_;

        double change_in_mean = (new_value - old_value) / window_size_;

        current_mean_ += change_in_mean;


        double change_in_variance = (new_value - old_value) * (new_value - current_mean_ + old_value - old_mean) / (window_size_ - 1);

        current_variance_ += change_in_variance;



    } else {
        // still adding to the queue, so just compute global stats

        std::deque<double>::iterator iter;


        double sum = 0;

        for (iter = queue_.begin(); iter != queue_.end(); iter++) {
            sum += *iter;
        }

        current_mean_ = sum/queue_.size();

        double var_sum = 0;

        for (iter = queue_.begin(); iter != queue_.end(); iter++) {
            var_sum += (*iter - current_mean_) * (*iter - current_mean_);
        }

        if (queue_.size() < 2) {
            current_variance_ = 0;
        } else {
            current_variance_ = var_sum / (queue_.size() - 1);
        }

    }

}


TEST(RollingStatstics, RollingStats) {

    RollingStatistics rolling_stats(5);

    rolling_stats.AddValue(10);

    EXPECT_NEAR( 10, rolling_stats.GetMean(), 0.00001 );
    EXPECT_NEAR( 0, rolling_stats.GetStandardDeviation(), 0.00001 );

    rolling_stats.AddValue(11);

    EXPECT_NEAR( 10.5, rolling_stats.GetMean(), 0.00001 );
    EXPECT_NEAR( 0.70710678118655, rolling_stats.GetStandardDeviation(), 0.00001);

    rolling_stats.AddValue(-3.4312);
    rolling_stats.AddValue(-4);
    rolling_stats.AddValue(0);

    EXPECT_NEAR (2.71376, rolling_stats.GetMean(), 0.00001);
    EXPECT_NEAR (7.2792627846507, rolling_stats.GetStandardDeviation(), 0.00001);

    rolling_stats.AddValue(43);

    EXPECT_NEAR (9.31376, rolling_stats.GetMean(), 0.00001);
    EXPECT_NEAR (19.773988638815, rolling_stats.GetStandardDeviation(), 0.00001);

    rolling_stats.AddValue(-0.11);
    EXPECT_NEAR (7.09176, rolling_stats.GetMean(), 0.00001);
    EXPECT_NEAR (20.157628419236, rolling_stats.GetStandardDeviation(), 0.00001);


}
