/*
 * Runs StereoBM (block matching stereo) on images coming from LCM.
 * Written so that we can do comparisons between StereoBM and our 3D
 * system.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */

#include "bm-stereo.hpp"


#define STEREO_DISTANCE_LIMIT 5.0 // meters


using namespace std;

lcm_t * lcm;

// globals for subscription functions, so we can unsubscribe in the control-c handler
bot_core_image_t_subscription_t *stereo_image_left_sub;
bot_core_image_t_subscription_t *stereo_image_right_sub;
lcmt_stereo_subscription_t *stereo_replay_sub;

mutex left_mutex, right_mutex, frame_number_mutex;

Mat left_image = Mat::zeros(240, 376, CV_8UC1);
Mat right_image = Mat::zeros(240, 376, CV_8UC1);

int frame_number, video_number;

int click_x = -1, click_y = -1;

bool new_frame_number = false, new_left = false, new_right = false;

int main(int argc,char** argv) {

    string config_file = "";

    ConciseArgs parser(argc, argv);
    parser.add(config_file, "c", "config", "Configuration file containing camera GUIDs, etc.", true);
    parser.parse();

    OpenCvStereoConfig stereo_config;

    // parse the config file
    if (ParseConfigFile(config_file, &stereo_config) != true)
    {
        fprintf(stderr, "Failed to parse configuration file, quitting.\n");
        return 1;
    }

    // load calibration
    OpenCvStereoCalibration stereo_calibration;

    if (LoadCalibration(stereo_config.calibrationDir, &stereo_calibration) != true)
    {
        cerr << "Error: failed to read calibration files. Quitting." << endl;
        return 1;
    }


    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    BotParam *param = bot_param_new_from_server(lcm, 0);
    if (param == NULL) {
        fprintf(stderr, "Error: no param server!\n");
        return 1;
    }

    // if a channel exists, subscribe to it

    char *stereo_replay_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_replay", &stereo_replay_channel) >= 0) {
        stereo_replay_sub = lcmt_stereo_subscribe(lcm, stereo_replay_channel, &stereo_replay_handler, NULL);
    }

    char *stereo_image_left_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_image_left", &stereo_image_left_channel) >= 0) {
        stereo_image_left_sub = bot_core_image_t_subscribe(lcm, stereo_image_left_channel, &stereo_image_left_handler, NULL);
    }

    char *stereo_image_right_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_image_right", &stereo_image_right_channel) >= 0) {
        stereo_image_right_sub = bot_core_image_t_subscribe(lcm, stereo_image_right_channel, &stereo_image_right_handler, NULL);
    }

    // control-c handler
    signal(SIGINT,sighandler);

    namedWindow("Left", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
    moveWindow("Left", 2700, 1000);

    namedWindow("Right", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
    moveWindow("Right", 2700+400, 1000);

    namedWindow("Disparity", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
    moveWindow("Disparity", 2700, 1000+350);

    namedWindow("Disparity3d", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
    moveWindow("Disparity3d", 2700+400, 1000+350);

    setMouseCallback("Disparity3d", onMouse); // for drawing disparity lines
    setMouseCallback("Disparity", onMouse);


    StereoBM *stereo_bm = new StereoBM(CV_STEREO_BM_BASIC);


    stereo_bm->state->preFilterSize 		= 105;
	stereo_bm->state->preFilterCap 		    = 61;
	stereo_bm->state->SADWindowSize 		= 5;
	stereo_bm->state->minDisparity 		    = 2;//36;
	stereo_bm->state->numberOfDisparities 	= 112;
	stereo_bm->state->textureThreshold  	= 427;
	stereo_bm->state->uniquenessRatio 	    = 18;
	stereo_bm->state->speckleWindowSize 	= 59;
	stereo_bm->state->speckleRange		    = 30;


    cout << "Running..." << endl;

    bool new_lcm = false;

    Mat disparity_bm = Mat::zeros(240, 376, CV_8UC1);
    Mat disparity_sgbm = Mat::zeros(240, 376, CV_8UC1);

    int this_frame_number;
    int this_video_number;

    while (true) {


        left_mutex.lock();
        right_mutex.lock();

        frame_number_mutex.lock();

        if (new_frame_number && new_left && new_right) {
            new_lcm = true;

            new_left = false;
            new_right = false;
            new_frame_number = false;
        }

        this_frame_number = frame_number;
        this_video_number = video_number;

        frame_number_mutex.unlock();


        // remap images
        Mat remap_left(left_image.rows, left_image.cols, left_image.depth());
        Mat remap_right(right_image.rows, right_image.cols, right_image.depth());

        remap(left_image, remap_left, stereo_calibration.mx1fp, Mat(), INTER_NEAREST);
        remap(right_image, remap_right, stereo_calibration.mx2fp, Mat(), INTER_NEAREST);

        imshow("Left", remap_left);
        imshow("Right", remap_right);

       // StereoSGBM sgbm_stereo(stereo_bm->state->minDisparity, stereo_bm->state->numberOfDisparities,
       //     stereo_bm->state->SADWindowSize);


        // do stereo processing
        if (new_lcm) {
            (*stereo_bm)(remap_left, remap_right, disparity_bm, CV_32F); // opencv overloads the () operator (function call), but we have a pointer, so we must dereference first.

            // run SGBM

          //  sgbm_stereo(remap_left, remap_right, disparity_sgbm);
        }

        Mat disp8, disp8_sgbm;
        disparity_bm.convertTo(disp8, CV_8U);
       // disparity_sgbm.convertTo(disp8_sgbm, CV_8U, 1.0/16.0);

        // project into 3d space
        Mat image_3d, image_3d_sgbm;
        reprojectImageTo3D(disparity_bm, image_3d, stereo_calibration.qMat, true);
       // reprojectImageTo3D(disparity_sgbm, image_3d_sgbm, stereo_calibration.qMat, true);


        // put these 3D coordinates in an LCM message
        lcmt_stereo msg;
        msg.timestamp = getTimestampNow();

        msg.frame_number = this_frame_number;
        msg.video_number = this_video_number;


        if (image_3d.isContinuous() == false) {
            cerr << "Error: image_3d not continuous." << endl;
            exit(1);
        }

        float *image_3d_ptr = image_3d.ptr<float>(0);
        float *image_3d_ptr_sgbm = image_3d_sgbm.ptr<float>(0);

        vector<float> x_vec, y_vec, z_vec;

        for (int i = 0; i < image_3d.cols * image_3d.rows * 3; i += 3) {

            if (image_3d_ptr[i+2] != 10000) {
                // this is a valid point

                // check to see if it is close enough for us to count
                if (abs(image_3d_ptr[i+2] / stereo_config.calibrationUnitConversion) < STEREO_DISTANCE_LIMIT) {
                    x_vec.push_back(image_3d_ptr[i] / stereo_config.calibrationUnitConversion);
                    y_vec.push_back(image_3d_ptr[i+1] / stereo_config.calibrationUnitConversion);
                    z_vec.push_back(image_3d_ptr[i+2] / stereo_config.calibrationUnitConversion);
                }
            }

      //      if (image_3d_ptr_sgbm[i+2] / stereo_config.calibrationUnitConversion > STEREO_DISTANCE_LIMIT) {
      //          image_3d_ptr_sgbm[i+2] = 10000;
      //          image_3d_ptr_sgbm[i+1] = 10000;
       //         image_3d_ptr_sgbm[i] = 10000;
//
       //     }
        }
        /*
        cv::vector<Point3f> debug_points, debug_points_3d;
        debug_points.push_back(Point3f(click_x, click_y, 0));

        perspectiveTransform(debug_points, debug_points_3d, stereo_calibration.qMat);

        x_vec.push_back(debug_points_3d[0].x);
        y_vec.push_back(debug_points_3d[0].y);
        z_vec.push_back(debug_points_3d[0].z);
        */


        msg.x = &x_vec[0];
        msg.y = &y_vec[0];
        msg.z = &z_vec[0];

        msg.number_of_points = x_vec.size() + 0; // TODO DEBUG



        if (new_lcm) {
            lcmt_stereo_publish(lcm, "stereo-bm", &msg);
            new_lcm = false;
        }

        // now strip out the values that are not at our disparity, so we can make a fair comparision
        // (and show the losses that come with our sparser technique)
        //Mat in_range;
        //inRange(disparity_bm, abs(state.disparity)-1, abs(state.disparity)+1, in_range);

        // display the disparity map
        //imshow("Debug 1", in_range);

        if (click_x > 0) {
            line(image_3d, Point(click_x, 0), Point(click_x, image_3d.rows), 128);
            line(image_3d, Point(0, click_y), Point(image_3d.cols, click_y), 128);

            line(disp8, Point(click_x, 0), Point(click_x, image_3d.rows), 128);
            line(disp8, Point(0, click_y), Point(image_3d.cols, click_y), 128);
        }

        // figure out details about where the mouse is clicking
        // specifically, get the disparity

        float this_disparity = disparity_bm.at<float>(click_y, click_x);


        vector<Point3f> click_point_vec;
        click_point_vec.push_back(Point3f(click_x, click_y, this_disparity));

        vector<Point3f> click_point_3d;

        perspectiveTransform(click_point_vec, click_point_3d, stereo_calibration.qMat);

        if (click_x > 0 && click_y > 0) {
            printf("\r(x, y): (%d, %d), disparity: %4.1f, (x, y, z): (%4.1f, %4.1f, %4.1f)      ",
                click_x, click_y, this_disparity, click_point_3d[0].x / stereo_config.calibrationUnitConversion, click_point_3d[0].y / stereo_config.calibrationUnitConversion, click_point_3d[0].z / stereo_config.calibrationUnitConversion);
            fflush(stdout);
        }



        imshow("Disparity", disp8);
       // imshow("Disparity SGBM", disp8_sgbm);



        imshow("Disparity3d", image_3d);
       // imshow("Disparity3d SGBM", image_3d_sgbm);








        right_mutex.unlock();
        left_mutex.unlock();


        char key = waitKey(1);

        if (key != 255 && key != -1)
        {
            cout << endl << key << endl;
        }

        switch (key)
        {
            case 'q':
                sighandler(0);
                break;
        }

        //lcm_handle(lcm);
        NonBlockingLcm(lcm);
    }

    return 0;
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}


void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user) {

    if (msg->pixelformat != 1196444237) { // PIXEL_FORMAT_MJPEG
        cerr << "Warning: reading images other than JPEG not yet implemented." << endl;
        return;
    }

    left_mutex.lock();

    left_image = Mat::zeros(msg->height, msg->width, CV_8UC1);

    // decompress JPEG
    jpeg_decompress_8u_gray(msg->data, msg->size, left_image.data, msg->width, msg->height, left_image.step);

    if (new_right) {
        cerr << "Warning: likely dropping a frame on the left image!" << endl;
    }

    new_left = true;

    left_mutex.unlock();

}

void stereo_image_right_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user) {

    if (msg->pixelformat != 1196444237) { // PIXEL_FORMAT_MJPEG
        cerr << "Warning: reading images other than JPEG not yet implemented." << endl;
        return;
    }

    right_mutex.lock();

    right_image = Mat::zeros(msg->height, msg->width, CV_8UC1);

    // decompress JPEG
    jpeg_decompress_8u_gray(msg->data, msg->size, right_image.data, msg->width, msg->height, right_image.step);

    if (new_right) {
        cerr << "Warning: likely dropping a frame on the right image!" << endl;
    }

    new_right = true;

    right_mutex.unlock();

}

// for replaying videos, subscribe to the stereo replay channel and set the frame number
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    frame_number_mutex.lock();

    frame_number = msg->frame_number;
    video_number = msg->video_number;

    if (new_frame_number) {
        cerr << "Warning: likely dropping a frame number!" << endl;
    }

    new_frame_number = true;

    frame_number_mutex.unlock();
}

/**
 * Mouse callback so that the user can click on an image
 */
void onMouse( int event, int x, int y, int flags, void* ) {
    if( flags & CV_EVENT_FLAG_LBUTTON)
    {
        // paint a line on the image they clicked on
        click_x = x;
        click_y = y;
    }
}


/**
 * Processes LCM messages without blocking.
 *
 * @param lcm lcm object
 *
 * @retval true if processed a message
 */
bool NonBlockingLcm(lcm_t *lcm)
{
    // setup an lcm function that won't block when we read it
    int lcm_fd = lcm_get_fileno(lcm);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);

    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        0,  // seconds
        1   // microseconds
    };


    int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

    if(0 == status) {
        // no messages
        //do nothing
        return false;

    } else if(FD_ISSET(lcm_fd, &fds)) {
        // LCM has events ready to be processed.
        lcm_handle(lcm);
        return true;
    }
    return false;

}

