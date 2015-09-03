#include "SpacialStereoFilter.hpp"

#define THRESHOLD 0.0001

SpacialStereoFilter::SpacialStereoFilter(float distance_threshold, int number_of_nearby_points_threshold) {
    distance_threshold_ = distance_threshold;

    // always include a point as a hit itself, which effectively means that the number
    // of points is reduced by one
    num_points_threshold_ = number_of_nearby_points_threshold - 1;
}

/**
 * Filters an LCM stereo message and returns a new message with
 * the reduced set of points
 *
 * @param msg LCM stereo message to be filtered
 *
 * @retval pointer to a new LCM stereo message that contains the filtered values.
 *      You must delete this message when you are done with it.
 *
 */
const lcmt::stereo* SpacialStereoFilter::ProcessMessage(const lcmt::stereo &msg) {
    lcmt::stereo *filtered_msg = new lcmt::stereo();

    filtered_msg->timestamp = msg.timestamp;
    filtered_msg->frame_number = msg.frame_number;
    filtered_msg->video_number = msg.video_number;

    if (msg.number_of_points < num_points_threshold_) {
        filtered_msg->number_of_points = 0;
        return filtered_msg;
    }

    std::vector<int> hit_counter(msg.number_of_points);

    for (int i = 0; i < msg.number_of_points; i++) {

        for (int j = i+1; j < msg.number_of_points; j++) {
            float dist = DistanceFunction(msg.x[i], msg.x[j], msg.y[i], msg.y[j], msg.z[i], msg.z[j]);
            if (dist <= distance_threshold_) {
                hit_counter[i] ++;
                hit_counter[j] ++;
            }
        }
    }

    int point_counter = 0;
    for (int i = 0; i < msg.number_of_points; i++) {
        if (hit_counter[i] >= num_points_threshold_) {
            filtered_msg->x.push_back(msg.x[i]);
            filtered_msg->y.push_back(msg.y[i]);
            filtered_msg->z.push_back(msg.z[i]);
            filtered_msg->grey.push_back(0);

            point_counter ++;
        }
    }

    filtered_msg->number_of_points = point_counter;

    return filtered_msg;
}

float SpacialStereoFilter::DistanceFunction(float x1, float x2, float y1, float y2, float z1, float z2) {
    //std::cout << "sqrt( ( " << x1 << " - " << x2 << ")^2 + (" << y1 << " - " << y2 << ")^2 + (" << z1 << " - " << z2 << ")^2  ) = " << sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)   ) << std::endl;
    return sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)   );
}


TEST(SpacialStereoFilterTest, Simple) {
    SpacialStereoFilter filter(1.0, 3);
}

TEST(SpacialStereoFilterTest, NoPoints) {
    SpacialStereoFilter filter(1.0, 3);

    lcmt::stereo msg;
    msg.timestamp = GetTimestampNow();
    msg.video_number = 3;
    msg.frame_number = 452;
    msg.number_of_points = 0;

    const lcmt::stereo *msg2;
    msg2 = filter.ProcessMessage(msg);

    ASSERT_TRUE(msg2->number_of_points == 0);
}

TEST(SpacialStereoFilterTest, OnePoint) {
    SpacialStereoFilter filter(1.0, 3);

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    msg.video_number = 3;
    msg.frame_number = 452;

    msg.x.push_back(0);
    msg.y.push_back(0);
    msg.z.push_back(0);
    msg.grey.push_back(0);

    msg.number_of_points = 1;

    const lcmt::stereo *msg2;
    msg2 = filter.ProcessMessage(msg);

    EXPECT_EQ_ARM(msg2->number_of_points, 0);
    delete msg2;

    msg.frame_number = 453;
    msg2 = filter.ProcessMessage(msg);

    ASSERT_TRUE(msg2->number_of_points == 0);
    delete msg2;
}

TEST(SpacialStereoFilterTest, AllHits) {
    SpacialStereoFilter filter(1.0, 3);

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    msg.video_number = 3;
    msg.frame_number = 452;

    msg.x.push_back(0);
    msg.y.push_back(0);
    msg.z.push_back(0);
    msg.grey.push_back(0);

    msg.x.push_back(0.1);
    msg.y.push_back(0.1);
    msg.z.push_back(0.1);
    msg.grey.push_back(0);

    msg.x.push_back(0.2);
    msg.y.push_back(0.2);
    msg.z.push_back(0.2);
    msg.grey.push_back(0);

    msg.number_of_points = 3;

    const lcmt::stereo *msg2;
    msg2 = filter.ProcessMessage(msg);

    ASSERT_TRUE(msg2->number_of_points == 3) << "Number of points = " << msg2->number_of_points;

    EXPECT_NEAR(msg2->x[0], 0, THRESHOLD);
    EXPECT_NEAR(msg2->y[0], 0, THRESHOLD);
    EXPECT_NEAR(msg2->z[0], 0, THRESHOLD);

    EXPECT_NEAR(msg2->x[1], 0.1, THRESHOLD);
    EXPECT_NEAR(msg2->y[1], 0.1, THRESHOLD);
    EXPECT_NEAR(msg2->z[1], 0.1, THRESHOLD);

    EXPECT_NEAR(msg2->x[2], 0.2, THRESHOLD);
    EXPECT_NEAR(msg2->y[2], 0.2, THRESHOLD);
    EXPECT_NEAR(msg2->z[2], 0.2, THRESHOLD);

    delete msg2;
}

TEST(SpacialStereoFilterTest, SomeHitsSomeMisses) {
    SpacialStereoFilter filter(1.0, 3);

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    msg.video_number = 3;
    msg.frame_number = 452;

    msg.x.push_back(0);
    msg.y.push_back(0);
    msg.z.push_back(0);
    msg.grey.push_back(0);

    msg.x.push_back(-0.1);
    msg.y.push_back(-0.1);
    msg.z.push_back(-0.5);
    msg.grey.push_back(0);

    msg.x.push_back(-0.2);
    msg.y.push_back(0);
    msg.z.push_back(0);
    msg.grey.push_back(0);

    msg.x.push_back(10);
    msg.y.push_back(0);
    msg.z.push_back(0);
    msg.grey.push_back(0);

    msg.number_of_points = 4;

    const lcmt::stereo *msg2;
    msg2 = filter.ProcessMessage(msg);

    ASSERT_TRUE(msg2->number_of_points == 3) << "Number of points = " << msg2->number_of_points;

    EXPECT_NEAR(msg2->x[0], 0, THRESHOLD);
    EXPECT_NEAR(msg2->y[0], 0, THRESHOLD);
    EXPECT_NEAR(msg2->z[0], 0, THRESHOLD);

    EXPECT_NEAR(msg2->x[1], -0.1, THRESHOLD);
    EXPECT_NEAR(msg2->y[1], -0.1, THRESHOLD);
    EXPECT_NEAR(msg2->z[1], -0.5, THRESHOLD);

    EXPECT_NEAR(msg2->x[2], -0.2, THRESHOLD);
    EXPECT_NEAR(msg2->y[2], 0, THRESHOLD);
    EXPECT_NEAR(msg2->z[2], 0, THRESHOLD);

    delete msg2;
}



//TEST(SpacialStereoFilterTest, NoHitPoints) {
    //StereoFilter filter(0.01);

    //lcmt::stereo msg;

    //msg.timestamp = GetTimestampNow();

    //msg.video_number = 1;
    //msg.frame_number = 0;

    //msg.x.push_back(3.232);
    //msg.y.push_back(10);
    //msg.z.push_back(12.13);

    //msg.number_of_points = 1;

    //const lcmt::stereo *msg2;
    //msg2 = filter.ProcessMessage(msg);
    //delete msg2;

    //msg2 = filter.ProcessMessage(msg);

    //ASSERT_TRUE(msg2->number_of_points == 1);
    //delete msg2;

    //msg.x[0] = 1;

    //msg.x.push_back(3.232);
    //msg.y.push_back(10);
    //msg.z.push_back(12.13);

    //msg.number_of_points = 2;

    //msg2 = filter.ProcessMessage(msg);

    //EXPECT_EQ_ARM(msg2->number_of_points, 1);
    //delete msg2;
//}

