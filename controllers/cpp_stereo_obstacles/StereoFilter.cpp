#include "StereoFilter.hpp"

StereoFilter::StereoFilter(float distance_threshold) {
    last_stereo_msg_ = NULL;
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
lcmt_stereo* StereoFilter::ProcessMessage(const lcmt_stereo *msg) {


    //cout << "Process message called" << endl << "=================================" << endl;

    // build a new stereo message to return

    lcmt_stereo *filtered_msg = new lcmt_stereo;

    filtered_msg->timestamp = msg->timestamp;
    filtered_msg->frame_number = msg->frame_number;
    filtered_msg->video_number = msg->video_number;

    float *x = new float[msg->number_of_points];
    float *y = new float[msg->number_of_points];
    float *z = new float[msg->number_of_points];

    filtered_msg->x = x;
    filtered_msg->y = y;
    filtered_msg->z = z;


    // check to see if filtering is possible
    if (last_stereo_msg_ == NULL) {
        filtered_msg->number_of_points = 0;


        last_stereo_msg_ = lcmt_stereo_copy(msg);
        return filtered_msg;
    }


    //PrintMsg(last_stereo_msg_, "last message");
    //PrintMsg(msg, "this message");



    if (msg->frame_number - last_stereo_msg_->frame_number < 0) {
        //cout << "JUMP BACK" << endl;
        delete last_stereo_msg_;
        last_stereo_msg_ = lcmt_stereo_copy(msg);
        return filtered_msg;
    }

    int point_counter = 0;

    // filtering is possible
    for (int i = 0; i < msg->number_of_points; i++) {

        if (FilterSinglePoint(msg->x[i], msg->y[i], msg->z[i])) {
            x[point_counter] = msg->x[i];
            y[point_counter] = msg->y[i];
            z[point_counter] = msg->z[i];

            point_counter ++;
        }
    }

    filtered_msg->number_of_points = point_counter;

    delete last_stereo_msg_;
    last_stereo_msg_ = lcmt_stereo_copy(msg);



    return filtered_msg;

}


bool StereoFilter::FilterSinglePoint(float x, float y, float z) {

    // check to see if this is near any of the points in the last message


    // NOTE: could implement an octree search if this linear search turns out to be too slow

    for (int i = 0; i < last_stereo_msg_->number_of_points; i++) {

        // compute distance to this point

        float dist = DistanceFunction(x, last_stereo_msg_->x[i], y, last_stereo_msg_->y[i], z, last_stereo_msg_->z[i]);

       // cout << "dist = " << dist << endl;
        if (dist < distance_threshold_) {

      //      cout << "(" << last_stereo_msg_->x[i] << ", " << x << ") , (" << last_stereo_msg_->y[i] << ", " << y << ") , (" << last_stereo_msg_->z[i] << ", " << z << ")" << endl;


            return true;
        }

    }

    return false;


}

float StereoFilter::DistanceFunction(float x1, float x2, float y1, float y2, float z1, float z2) {
    //cout << "sqrt( ( " << x1 << " - " << x2 << ")^2 + (" << y1 << " - " << y2 << ")^2 + (" << z1 << " - " << z2 << ")^2  ) = " << sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)   ) << endl;
    return sqrt(  (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)   );
}

void StereoFilter::PrintMsg(const lcmt_stereo *msg, string header) {
    cout << "-------------- " << header << " -------------" << endl << "Timestamp: " << msg->timestamp;
    cout << endl << "Frame number: " << msg->frame_number << endl << "number_of_points: " << msg->number_of_points;
    cout << endl;

    for (int i = 0; i < msg->number_of_points; i++) {
        cout << "(" << msg->x[i] << ", " << msg->y[i] << ", " << msg->z[i] << ")" << endl;
    }
    cout << "------------ end message ---------------" << endl;
}
