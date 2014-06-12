/*
 * Displays plane HUD along with results from octomap.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */

#include "hud-main.hpp"

using namespace std;

lcm_t * lcm;

// globals for subscription functions, so we can unsubscribe in the control-c handler
mav_pose_t_subscription_t *mav_pose_t_sub;
lcmt_baro_airspeed_subscription_t *baro_airspeed_sub;
lcmt_battery_status_subscription_t *battery_status_sub;
lcmt_deltawing_u_subscription_t *servo_out_sub;
mav_gps_data_t_subscription_t *mav_gps_data_t_sub;
bot_core_image_t_subscription_t *stereo_image_left_sub;
lcmt_stereo_subscription_t *stereo_replay_sub;
lcmt_stereo_subscription_t *stereo_sub;
lcmt_stereo_subscription_t *stereo_bm_sub;
octomap_raw_t_subscription_t *octomap_sub;

mutex image_mutex;
Mat left_image = Mat::zeros(240, 376, CV_8UC1); // global so we can update it in the stereo handler and in the main loop


mutex stereo_mutex, stereo_bm_mutex;
lcmt_stereo *last_stereo_msg, *last_stereo_bm_msg;

OcTree *octree = NULL;
mutex octomap_mutex;

int main(int argc,char** argv) {
    
    string config_file = "";
    int move_window_x = -1, move_window_y = -1;
    
    ConciseArgs parser(argc, argv);
    parser.add(config_file, "c", "config", "Configuration file containing camera GUIDs, etc.", true);
    parser.add(move_window_x, "x", "move-window-x", "Move window starting location x (must pass both x and y)");
    parser.add(move_window_y, "y", "move-window-y", "Move window starting location y (must pass both x and y)");
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
    
    BotFrames *bot_frames = bot_frames_new(lcm, param);
    
    // create a HUD object so we can pass it's pointer to the lcm handlers
    Hud hud;

    // if a channel exists, subscribe to it
    char *pose_channel;
    if (bot_param_get_str(param, "coordinate_frames.body.pose_update_channel", &pose_channel) >= 0) {
        mav_pose_t_sub = mav_pose_t_subscribe(lcm, pose_channel, &mav_pose_t_handler, &hud);
    }
    
    char *gps_channel;
    if (bot_param_get_str(param, "lcm_channels.gps", &gps_channel) >= 0) {
        mav_gps_data_t_sub = mav_gps_data_t_subscribe(lcm, gps_channel, &mav_gps_data_t_handler, &hud);
    }
    
    char *baro_airspeed_channel;
    if (bot_param_get_str(param, "lcm_channels.baro_airspeed", &baro_airspeed_channel) >= 0) {
        baro_airspeed_sub = lcmt_baro_airspeed_subscribe(lcm, baro_airspeed_channel, &baro_airspeed_handler, &hud);
    }
    
    char *servo_out_channel;
    if (bot_param_get_str(param, "lcm_channels.servo_out", &servo_out_channel) >= 0) {
        servo_out_sub = lcmt_deltawing_u_subscribe(lcm, servo_out_channel, &servo_out_handler, &hud);
    }
    
    char *battery_status_channel;
    if (bot_param_get_str(param, "lcm_channels.battery_status", &battery_status_channel) >= 0) {
        battery_status_sub = lcmt_battery_status_subscribe(lcm, battery_status_channel, &battery_status_handler, &hud);
    }
    
    char *stereo_replay_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_replay", &stereo_replay_channel) >= 0) {
        stereo_replay_sub = lcmt_stereo_subscribe(lcm, stereo_replay_channel, &stereo_replay_handler, &hud);
    }
    
    char *stereo_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo", &stereo_channel) >= 0) {
        stereo_sub = lcmt_stereo_subscribe(lcm, stereo_channel, &stereo_handler, NULL);
    }
    
    char *stereo_image_left_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_image_left", &stereo_image_left_channel) >= 0) {
        stereo_image_left_sub = bot_core_image_t_subscribe(lcm, stereo_image_left_channel, &stereo_image_left_handler, &hud);
    }
    
    char *octomap_channel;
    if (bot_param_get_str(param, "lcm_channels.octomap", &octomap_channel) >= 0) {
        octomap_sub = octomap_raw_t_subscribe(lcm, octomap_channel, &octomap_raw_t_handler, NULL);
    }
    
    char *stereo_bm_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_bm", &stereo_bm_channel) >= 0) {
        stereo_bm_sub = lcmt_stereo_subscribe(lcm, stereo_bm_channel, &stereo_bm_handler, NULL);
    }
    
    // control-c handler
    signal(SIGINT,sighandler);
    
    namedWindow("HUD", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
    
    if (move_window_x != -1 && move_window_y != -1) {
        moveWindow("HUD", move_window_x, move_window_y);
    }
    
    cout << "Running..." << endl;

    while (true) {
        // read the LCM channel, but process everything to allow us to drop frames
        while (NonBlockingLcm(lcm)) {}
        
        
        
        Mat hud_image, temp_image;
        
        image_mutex.lock();
        left_image.copyTo(temp_image);
        image_mutex.unlock();
        
        
        vector<Point3f> octomap_points;
        
        BotTrans global_to_body;
        bot_frames_get_trans(bot_frames, "local", "opencvFrame", &global_to_body);
        
        if (GetOctomapPoints(&octomap_points, &global_to_body)) {
            
            //cout << "drawing" << octomap_points << endl;
            
            Draw3DPointsOnImage(temp_image, &octomap_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, 128);
        }
        
        
        // transform the point from 3D space back onto the image's 2D space
        vector<Point3f> lcm_points;
        
        stereo_mutex.lock();
        if (last_stereo_msg) {
            Get3DPointsFromStereoMsg(last_stereo_msg, &lcm_points);
        }
        stereo_mutex.unlock();

        //cout << lcm_points << endl;

        Draw3DPointsOnImage(temp_image, &lcm_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, 0);
        
        vector<Point3f> bm_points;
        stereo_bm_mutex.lock();
        if (last_stereo_bm_msg) {
            Get3DPointsFromStereoMsg(last_stereo_bm_msg, &bm_points);
        }
        stereo_bm_mutex.unlock();
        Draw3DPointsOnImage(temp_image, &bm_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, 128, 0);
        
        
        // remap
        Mat remapped_image;
        remap(temp_image, remapped_image, stereo_calibration.mx1fp, Mat(), INTER_NEAREST);
        
        hud.DrawHud(remapped_image, hud_image);
        
    
        imshow("HUD", hud_image);
        
        
        
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

bool GetOctomapPoints(vector<Point3f> *octomap_points, BotTrans *global_to_body) {
    octomap_mutex.lock();
    
    if (octree == NULL) {
        octomap_mutex.unlock();
        return false;
    }
    
    
    // loop through the most likely points on the octomap and plot them on the image
    for(OcTree::leaf_iterator it = octree->begin_leafs(), end=octree->end_leafs(); it!= end; ++it) {
        //manipulate node, e.g.:
        
        // check to see if this is occupied
        if (it->getOccupancy() > 0.2) {
        
            octomap::point3d this_point = it.getCoordinate();
            
            // convert this global coordinate into the local coordinate frame
            double this_point_d[3];
            this_point_d[0] = this_point.x();
            this_point_d[1] = this_point.y();
            this_point_d[2] = this_point.z();
            
            double point_in_body_coords[3];
            bot_trans_apply_vec(global_to_body, this_point_d, point_in_body_coords);
                
            // don't plot points that are behind us
            if (point_in_body_coords[2] >= 0) {
                octomap_points->push_back(Point3f(point_in_body_coords[0], point_in_body_coords[1], point_in_body_coords[2]));
            }
            
        }
    }
    
    octomap_mutex.unlock();
    
    return true;
    
}



void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    stereo_mutex.lock();
    if (last_stereo_msg) {
        delete last_stereo_msg;
    }
    
    last_stereo_msg = lcmt_stereo_copy(msg);
    stereo_mutex.unlock();
}

void stereo_bm_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    stereo_bm_mutex.lock();
    if (last_stereo_bm_msg) {
        delete last_stereo_bm_msg;
    }
    
    last_stereo_bm_msg = lcmt_stereo_copy(msg);
    stereo_bm_mutex.unlock();
}

void stereo_image_left_handler(const lcm_recv_buf_t *rbuf, const char* channel, const bot_core_image_t *msg, void *user) {
    
    if (msg->pixelformat != 1196444237) { // PIXEL_FORMAT_MJPEG
        cerr << "Warning: reading images other than JPEG not yet implemented." << endl;
        return;
    }
    
    image_mutex.lock();
    
    left_image = Mat::zeros(msg->height, msg->width, CV_8UC1);
    
    // decompress JPEG
    jpeg_decompress_8u_gray(msg->data, msg->size, left_image.data, msg->width, msg->height, left_image.step);
    
    image_mutex.unlock();
    
}

// for replaying videos, subscribe to the stereo replay channel and set the frame number
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    Hud *hud = (Hud*)user;
    
    hud->SetFrameNumber(msg->frame_number);
    hud->SetVideoNumber(msg->video_number);
}

void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user) {
    Hud *hud = (Hud*)user;
    
    hud->SetAirspeed(msg->airspeed);
}

void battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user) {
    Hud *hud = (Hud*)user;
    
    hud->SetBatteryVoltage(msg->voltage);
}

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user) {
    
    Hud *hud = (Hud*)user;
    
    hud->SetServoCommands((msg->throttle - 1100) * 100/797, (msg->elevonL-1000)/10.0, (msg->elevonR-1000)/10.0);
    hud->SetAutonomous(msg->is_autonomous);
}

void mav_gps_data_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user) {
    Hud *hud = (Hud*)user;
    
    hud->SetGpsSpeed(msg->speed);
    hud->SetGpsHeading(msg->heading);
}

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {
    Hud *hud = (Hud*)user;
    
    hud->SetAltitude(msg->pos[2]);
    hud->SetOrientation(msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3]);
    hud->SetAcceleration(msg->accel[0], msg->accel[1], msg->accel[2]);
    
    hud->SetTimestamp(msg->utime);
}


void octomap_raw_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const octomap_raw_t *msg, void *user) {
    // get an octomap and load it into memory
    
    octomap_mutex.lock();
    
    if (octree) {
        delete octree;
    }
    
    std::stringstream datastream;
    datastream.write((const char*) msg->data, msg->length);
    
    octree = new octomap::OcTree(1); //resolution will be set by data from message
    octree->readBinary(datastream);
    
    octree->toMaxLikelihood();
    
    octomap_mutex.unlock();
    
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


