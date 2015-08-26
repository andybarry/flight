#include "StereoFilter.hpp"

StereoFilter::StereoFilter(float distance_threshold) {
    last_stereo_msg_ = nullptr;
    distance_threshold_ = distance_threshold;

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
const lcmt::stereo* StereoFilter::ProcessMessage(const lcmt::stereo &msg) {


    //std::cout << "Process message called" << std::endl << "=================================" << std::endl;

    // build a new stereo message to return

    lcmt::stereo *filtered_msg = new lcmt::stereo();

    filtered_msg->timestamp = msg.timestamp;
    filtered_msg->frame_number = msg.frame_number;
    filtered_msg->video_number = msg.video_number;


    // check to see if filtering is possible
    if (last_stereo_msg_ == nullptr) {
        filtered_msg->number_of_points = 0;


        last_stereo_msg_ = new lcmt::stereo(msg);
        return filtered_msg;
    }


    //PrintMsg(last_stereo_msg_, "last message");
    //PrintMsg(msg, "this message");

    if (msg.frame_number - last_stereo_msg_->frame_number < 0) {
        //std::cout << "JUMP BACK" << std::endl;
        delete last_stereo_msg_;
        last_stereo_msg_ = new lcmt::stereo(msg);
        return filtered_msg;
    }

    int point_counter = 0;

    // filtering is possible
    for (int i = 0; i < msg.number_of_points; i++) {

        if (FilterSinglePoint(msg.x[i], msg.y[i], msg.z[i])) {
            filtered_msg->x.push_back(msg.x[i]);
            filtered_msg->y.push_back(msg.y[i]);
            filtered_msg->z.push_back(msg.z[i]);

            point_counter ++;
        }
    }

    filtered_msg->number_of_points = point_counter;

    delete last_stereo_msg_;
    last_stereo_msg_ = new lcmt::stereo(msg);



    return filtered_msg;

}


bool StereoFilter::FilterSinglePoint(float x, float y, float z) {

    // check to see if this is near any of the points in the last message
    // NOTE: could implement an octree search if this linear search turns out to be too slow
    for (int i = 0; i < last_stereo_msg_->number_of_points; i++) {
        // compute distance to this point
        float dist = DistanceFunction(x, last_stereo_msg_->x[i], y, last_stereo_msg_->y[i], z, last_stereo_msg_->z[i]);
       // std::cout << "dist = " << dist << std::endl;
        if (dist < distance_threshold_) {

      //      std::cout << "(" << last_stereo_msg_->x[i] << ", " << x << ") , (" << last_stereo_msg_->y[i] << ", " << y << ") , (" << last_stereo_msg_->z[i] << ", " << z << ")" << std::endl;
            return true;
        }
    }
    return false;


}

float StereoFilter::DistanceFunction(float x1, float x2, float y1, float y2, float z1, float z2) {
    //std::cout << "sqrt( ( " << x1 << " - " << x2 << ")^2 + (" << y1 << " - " << y2 << ")^2 + (" << z1 << " - " << z2 << ")^2  ) = " << sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)   ) << std::endl;
    return sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)   );
}

void StereoFilter::PrintMsg(const lcmt::stereo &msg, std::string header) const {
    std::cout << "-------------- " << header << " -------------" << std::endl << "Timestamp: " << msg.timestamp;
    std::cout << std::endl << "Frame number: " << msg.frame_number << std::endl << "number_of_points: " << msg.number_of_points;
    std::cout << std::endl;

    for (int i = 0; i < msg.number_of_points; i++) {
        std::cout << "(" << msg.x[i] << ", " << msg.y[i] << ", " << msg.z[i] << ")" << std::endl;
    }
    std::cout << "------------ end message ---------------" << std::endl;
}

TEST(StereoFilterTest, Simple) {
    StereoFilter filter(0.01);
}

TEST(StereoFilterTest, OnePoint) {
    StereoFilter filter(0.01);

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    msg.video_number = 3;
    msg.frame_number = 452;

    msg.x.push_back(0);
    msg.y.push_back(0);
    msg.z.push_back(0);

    msg.number_of_points = 1;

    const lcmt::stereo *msg2;
    msg2 = filter.ProcessMessage(msg);

    EXPECT_EQ_ARM(msg2->number_of_points, 0);
    delete msg2;

    msg.frame_number = 453;
    msg2 = filter.ProcessMessage(msg);

    ASSERT_TRUE(msg2->number_of_points == 1);

    EXPECT_EQ_ARM(msg2->x[0], 0);
    EXPECT_EQ_ARM(msg2->y[0], 0);
    EXPECT_EQ_ARM(msg2->z[0], 0);
}

TEST(StereoFilterTest, NoHitPoints) {
    StereoFilter filter(0.01);

    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    msg.video_number = 1;
    msg.frame_number = 0;

    msg.x.push_back(3.232);
    msg.y.push_back(10);
    msg.z.push_back(12.13);

    msg.number_of_points = 1;

    const lcmt::stereo *msg2;
    msg2 = filter.ProcessMessage(msg);
    delete msg2;

    msg2 = filter.ProcessMessage(msg);

    ASSERT_TRUE(msg2->number_of_points == 1);
    delete msg2;

    msg.x[0] = 1;

    msg.x.push_back(3.232);
    msg.y.push_back(10);
    msg.z.push_back(12.13);

    msg.number_of_points = 2;

    msg2 = filter.ProcessMessage(msg);

    EXPECT_EQ_ARM(msg2->number_of_points, 1);
    delete msg2;
}

