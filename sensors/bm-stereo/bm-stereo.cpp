/*
 * Runs StereoBM (block matching stereo) on images coming from LCM.
 * Written so that we can do comparisons between StereoBM and our 3D
 * system.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */

#include "bm-stereo.hpp"

using namespace std;

lcm_t * lcm;

// globals for subscription functions, so we can unsubscribe in the control-c handler
bot_core_image_t_subscription_t *stereo_image_left_sub;
bot_core_image_t_subscription_t *stereo_image_right_sub;
lcmt_stereo_subscription_t *stereo_replay_sub;

mutex left_mutex, right_mutex;

Mat left_image = Mat::zeros(240, 376, CV_8UC1);
Mat right_image = Mat::zeros(240, 376, CV_8UC1);

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
    
    
    
    StereoBM *stereo_bm = new StereoBM(CV_STEREO_BM_BASIC);
    
    cout << "Running..." << endl;

    while (true) {
        
        left_mutex.lock();
        right_mutex.lock();
        
        cout << "locked..." << endl;


        // remap images
        Mat remap_left(left_image.rows, left_image.cols, left_image.depth());
        Mat remap_right(right_image.rows, right_image.cols, right_image.depth());
        cout << "remap..." << endl;
        remap(left_image, remap_left, stereo_calibration.mx1fp, Mat(), INTER_NEAREST);
        remap(right_image, remap_right, stereo_calibration.mx2fp, Mat(), INTER_NEAREST);
        cout << "remap done..." << endl;
        imshow("Left", remap_left);
        imshow("Right", remap_right);
        
        cout << "shown..." << endl;
        // do stereo processing
        Mat disparity_bm;
        (*stereo_bm)(remap_left, remap_right, disparity_bm, CV_32F); // opencv overloads the () operator (function call), but we have a pointer, so we must dereference first.
        cout << "stereo done..." << endl;
        Mat disp8;
        disparity_bm.convertTo(disp8, CV_8U, 255/(stereo_bm->state->numberOfDisparities*16.));
        cout << "stereo done and converted..." << endl;
        // project into 3d space
        Mat image_3d;
        reprojectImageTo3D(disparity_bm, image_3d, stereo_calibration.qMat, true);
        
        
        // put these 3D coordinates in an LCM message
        lcmt_stereo msg;
        msg.timestamp = getTimestampNow();
        
        msg.frame_number = -1; // TODO
        
        
        if (image_3d.isContinuous() == false) {
            cerr << "Error: image_3d not continuous." << endl;
            exit(1);
        }
        
        float *image_3d_ptr = image_3d.ptr<float>(0);
        
        vector<float> x_vec, y_vec, z_vec;
        
        
        cout << "2..." << endl;
        for (int i = 0; i < image_3d.cols * image_3d.rows; i += 3) {
            
            if (image_3d_ptr[i+2] < 10000) {
                // this is a valid point
                
                x_vec.push_back(image_3d_ptr[i]);
                y_vec.push_back(image_3d_ptr[i+1]);
                z_vec.push_back(image_3d_ptr[i+2]);
                
                
            }
        }
        
        
        cout << "3..." << endl;
        msg.x = &x_vec[0];
        msg.y = &y_vec[0];
        msg.z = &z_vec[0];
        
        msg.number_of_points = x_vec.size();
cout << "4..." << endl;
        lcmt_stereo_publish(lcm, "stereo-bm", &msg);
        cout << "published..." << endl;
    
        
        // now strip out the values that are not at our disparity, so we can make a fair comparision
        // (and show the losses that come with our sparser technique)
        //Mat in_range;
        //inRange(disparity_bm, abs(state.disparity)-1, abs(state.disparity)+1, in_range);
        
        // display the disparity map
        //imshow("Debug 1", in_range);
        imshow("Disparity", disp8);
        imshow("Disparity3d", image_3d);
        
        
        
        
        
        
        
        
        
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
        
        lcm_handle(lcm);
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
    
    right_mutex.unlock();
    
}

// for replaying videos, subscribe to the stereo replay channel and set the frame number
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
}


