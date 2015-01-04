/*
 * Displays plane HUD along with results from octomap.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2014
 *
 */

#include "hud-main.hpp"

#define BM_DEPTH_MIN 4.7
#define BM_DEPTH_MAX 4.9

using namespace std;

lcm_t * lcm;

// globals for subscription functions, so we can unsubscribe in the control-c handler
mav_pose_t_subscription_t *mav_pose_t_sub;
mav_pose_t_subscription_t *mav_pose_t_replay_sub;
lcmt_baro_airspeed_subscription_t *baro_airspeed_sub;
lcmt_battery_status_subscription_t *battery_status_sub;
lcmt_deltawing_u_subscription_t *servo_out_sub;
mav_gps_data_t_subscription_t *mav_gps_data_t_sub;
bot_core_image_t_subscription_t *stereo_image_left_sub;
lcmt_stereo_subscription_t *stereo_replay_sub;
lcmt_stereo_subscription_t *stereo_sub;
lcmt_stereo_with_xy_subscription_t *stereo_xy_sub;
lcmt_stereo_subscription_t *stereo_bm_sub;
octomap_raw_t_subscription_t *octomap_sub;

mutex image_mutex;
Mat left_image = Mat::zeros(240, 376, CV_8UC1); // global so we can update it in the stereo handler and in the main loop

ofstream box_file;

mutex stereo_mutex, stereo_bm_mutex, stereo_xy_mutex, ui_box_mutex, stereo_replay_mutex;
lcmt_stereo *last_stereo_msg, *last_stereo_bm_msg, *last_stereo_replay_msg;
lcmt_stereo_with_xy *last_stereo_xy_msg;

OcTree *octree = NULL;
mutex octomap_mutex;

bool ui_box = false;
bool ui_box_first_click = false;
bool ui_box_done = false;

bool real_frame_loaded = false;


Point2d box_top(-1, -1);
Point2d box_bottom(-1, -1);

int main(int argc,char** argv) {

    Scalar bm_color(.5, .5, .5);
    Scalar block_match_color(.8, 0, 0);
    Scalar block_match_fill_color(1, 1, 1);
    Scalar octomap_color(1, 0, .4);

    string config_file = "";
    int move_window_x = -1, move_window_y = -1;
    bool replay_hud_bool = false;
    bool record_hud = false;
    bool show_unremapped = false;
    bool depth_crop_bm = false;
    bool draw_stereo_replay = false;
    int clutter_level = 5;
    string ui_box_path = ""; // a mode that lets the user draw boxes on screen to select relevant parts of the image


    ConciseArgs parser(argc, argv);
    parser.add(config_file, "c", "config", "Configuration file containing camera GUIDs, etc.", true);
    parser.add(move_window_x, "x", "move-window-x", "Move window starting location x (must pass both x and y)");
    parser.add(move_window_y, "y", "move-window-y", "Move window starting location y (must pass both x and y)");
    parser.add(replay_hud_bool, "r", "replay-hud", "Enable a second HUD on the channel STATE_ESTIMATOR_POSE_REPLAY");
    parser.add(clutter_level, "C", "clutter-level", "Sets clutter level for HUD display from 0 (just image) to 5 (full HUD)");
    parser.add(record_hud, "R", "record-hud", "Enable recording to disk.");
    parser.add(show_unremapped, "u", "show-unremapped", "Show the unremapped image");
    parser.add(ui_box_path, "b", "draw-box", "Path to write box drawing results to.");
    parser.add(depth_crop_bm, "z", "depth-crop-bm", "Crop depth for BM stereo to between BM_DEPTH_MIN and BM_DEPTH_MAX meters.");
    parser.add(draw_stereo_replay, "s", "draw_stereo_replay", "Display stereo points from the stereo_replay channel.");
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

    if (ui_box_path != "") {
        // init box parsing

        // open a file to write to
        ui_box = true;
        show_unremapped = true;
        box_file.open(ui_box_path);
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

    if (clutter_level < 0 || clutter_level > 5) {
        fprintf(stderr, "Error: clutter level out of bounds.\n");
        return 1;
    }

    float bm_depth_min = 0, bm_depth_max = 0;

    if (depth_crop_bm) {
        bm_depth_min = BM_DEPTH_MIN;
        bm_depth_max = BM_DEPTH_MAX;
    }

    BotFrames *bot_frames = bot_frames_new(lcm, param);

    RecordingManager recording_manager;
    recording_manager.Init(stereo_config);

    // create a HUD object so we can pass it's pointer to the lcm handlers
    Hud hud;
    hud.SetClutterLevel(clutter_level);
    Hud replay_hud(Scalar(0, 0, 0.8));
    replay_hud.SetClutterLevel(99);
    replay_hud.SetImageScaling(1);

    // if a channel exists, subscribe to it
    char *pose_channel;
    if (bot_param_get_str(param, "coordinate_frames.body.pose_update_channel", &pose_channel) >= 0) {
        mav_pose_t_sub = mav_pose_t_subscribe(lcm, pose_channel, &mav_pose_t_handler, &hud);
    }

    if (replay_hud_bool) {
        mav_pose_t_replay_sub = mav_pose_t_subscribe(lcm, "STATE_ESTIMATOR_POSE_REPLAY", &mav_pose_t_handler, &replay_hud); // we can use the same handler, and just send it the replay_hud pointer
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

    char *stereo_xy_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo_with_xy", &stereo_xy_channel) >= 0) {
        stereo_xy_sub = lcmt_stereo_with_xy_subscribe(lcm, stereo_xy_channel, &stereo_xy_handler, NULL);
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
    setMouseCallback("HUD", OnMouse, &hud);

    if (move_window_x != -1 && move_window_y != -1) {
        moveWindow("HUD", move_window_x, move_window_y);
    }

    cout << "Running..." << endl;

    bool change_flag = true;
    bool real_frame_loaded_and_run;

    Mat hud_image;

    while (true) {
        // read the LCM channel, but process everything to allow us to drop frames
        while (NonBlockingLcm(lcm)) {
            change_flag = true;
        }

        if (change_flag == true || ui_box) {
            change_flag = false;
            real_frame_loaded_and_run = false;

            Mat gray_img;

            image_mutex.lock();
            if (real_frame_loaded) {
                real_frame_loaded_and_run = true;
            }
            left_image.copyTo(gray_img);
            image_mutex.unlock();

            Mat color_img;

            if (gray_img.type() == CV_8UC1) {
                // convert to color
                Mat gray_img2;
                gray_img.convertTo(gray_img2, CV_32FC3, 1/255.0);


                cvtColor(gray_img2, color_img, CV_GRAY2BGR);
            } else if (gray_img.type() == CV_8UC3) {

                gray_img.convertTo(color_img, CV_32FC3, 1/255.0);

            } else if (gray_img.type() == CV_32FC3) {
                color_img = gray_img;

            } else {
                cout << "Error: unsupported image type." << endl;
                return -1;
            }

            // -- BM stereo -- //
            vector<Point3f> bm_points;
            stereo_bm_mutex.lock();
            if (last_stereo_bm_msg) {
                Get3DPointsFromStereoMsg(last_stereo_bm_msg, &bm_points);
            }
            stereo_bm_mutex.unlock();

            vector<int> valid_bm_points;

            if (box_bottom.x == -1) {
                Draw3DPointsOnImage(color_img, &bm_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, bm_color, 0, Point2d(-1, -1), Point2d(-1, -1), NULL, bm_depth_min, bm_depth_max);
            } else {
                 Draw3DPointsOnImage(color_img, &bm_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, bm_color, 0, box_top, box_bottom, &valid_bm_points, bm_depth_min, bm_depth_max);
             }


            // -- octomap -- //
            vector<Point3f> octomap_points;

            BotTrans global_to_body;
            bot_frames_get_trans(bot_frames, "local", "opencvFrame", &global_to_body);

            octomap_mutex.lock();
            if (octree != NULL) {
                StereoOctomap::GetOctomapPoints(octree, &octomap_points, &global_to_body, true);

                //Draw3DPointsOnImage(color_img, &octomap_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, 128);
            }
            octomap_mutex.unlock();

            // -- stereo -- //

            // transform the point from 3D space back onto the image's 2D space
            vector<Point3f> lcm_points;

            stereo_mutex.lock();
            if (last_stereo_msg) {
                Get3DPointsFromStereoMsg(last_stereo_msg, &lcm_points);
            }
            stereo_mutex.unlock();

            Draw3DPointsOnImage(color_img, &lcm_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, block_match_color, block_match_fill_color);


            // -- stereo replay -- //

            if (draw_stereo_replay) {
                // transform the point from 3D space back onto the image's 2D space
                vector<Point3f> lcm_replay_points;

                stereo_replay_mutex.lock();
                if (last_stereo_replay_msg) {
                    Get3DPointsFromStereoMsg(last_stereo_replay_msg, &lcm_replay_points);
                }
                stereo_replay_mutex.unlock();

                Draw3DPointsOnImage(color_img, &lcm_replay_points, stereo_calibration.M1, stereo_calibration.D1, stereo_calibration.R1, Scalar(1, 1, 1), 0);
            }

            // -- octomap XY -- //
            stereo_xy_mutex.lock();
            if (last_stereo_xy_msg) {
                vector<Point> xy_points;

                Get2DPointsFromLcmXY(last_stereo_xy_msg, &xy_points);
                Draw2DPointsOnImage(color_img, &xy_points);
            }
            stereo_xy_mutex.unlock();


            // -- box ui -- //

            // check for box_ui management


            if (ui_box_done) {

                // the box is done

                // write to a file, update variables, and ask for a new frame

                box_file << hud.GetVideoNumber() << "," << hud.GetFrameNumber() << "," <<
                    box_top.x << "," << box_top.y << "," << box_bottom.x << "," << box_bottom.y;
                cout << hud.GetVideoNumber() << "," << hud.GetFrameNumber() << "," <<
                    box_top.x << "," << box_top.y << "," << box_bottom.x << "," << box_bottom.y;

                for (int valid : valid_bm_points) {
                    box_file << "," << valid;
                    cout << "," << valid;
                }

                box_file << endl;
                cout << endl;

                // request a new frame
                AskForFrame(hud.GetVideoNumber(), hud.GetFrameNumber() + 1);

                ResetBoxDrawing();

            }
            ui_box_mutex.unlock();


            // remap
            Mat remapped_image;
            if (show_unremapped == false) {
                remap(color_img, remapped_image, stereo_calibration.mx1fp, Mat(), INTER_NEAREST);
            } else {
                color_img.copyTo(remapped_image);
            }

            if (ui_box) {
                if (box_bottom.x == -1) {
                    line(remapped_image, Point(box_top.x, 0), Point(box_top.x, remapped_image.rows), 0);
                    line(remapped_image, Point(0, box_top.y), Point(remapped_image.cols, box_top.y), 0);
                } else {
                    rectangle(remapped_image, box_top, box_bottom, 128);
                }
            }

            hud.DrawHud(remapped_image, hud_image);

            if (replay_hud_bool) {
                Mat temp;
                hud_image.copyTo(temp);
                replay_hud.DrawHud(temp, hud_image);
            }


            if (record_hud && real_frame_loaded_and_run) {
                // put this frame into the HUD recording
                recording_manager.RecFrameHud(hud_image);
            }

            imshow("HUD", hud_image);

        }



        char key = waitKey(1);

        if (key != 255 && key != -1) {
            cout << endl << key << endl;
        }

        switch (key) {
            case 'q':
                sighandler(0);
                break;

            case ',':
                AskForFrame(hud.GetVideoNumber(), hud.GetFrameNumber() - 1);
                ResetBoxDrawing();
                break;

            case '<':
                AskForFrame(hud.GetVideoNumber(), hud.GetFrameNumber() - 50);
                ResetBoxDrawing();
                break;

            case '.':
                AskForFrame(hud.GetVideoNumber(), hud.GetFrameNumber() + 1);
                ResetBoxDrawing();
                break;

            case '>':
                AskForFrame(hud.GetVideoNumber(), hud.GetFrameNumber() + 50);
                ResetBoxDrawing();
                break;
            case 'R':
                record_hud = true;
                recording_manager.RestartRecHud();
                break;

            case 'c':
                hud.SetClutterLevel(hud.GetClutterLevel() + 1);
                change_flag = true;
                break;

            case 'C':
                hud.SetClutterLevel(hud.GetClutterLevel() - 1);
                change_flag = true;
                break;

            case 's':
                draw_stereo_replay = !draw_stereo_replay;
                change_flag = true;
                break;

            case 'z':
                depth_crop_bm = !depth_crop_bm;

                if (depth_crop_bm) {
                    bm_depth_min = BM_DEPTH_MIN;
                    bm_depth_max = BM_DEPTH_MAX;
                } else {
                    bm_depth_min = 0;
                    bm_depth_max = 0;
                }
                change_flag = true;

                break;


            case 'S':
                // take a screen cap
                printf("\nWriting hud.png...");

                Mat converted_hud, converted_hud2;

                hud_image.convertTo(converted_hud, CV_16U, 65536);

                imwrite("hud.png", converted_hud);
                printf("\ndone.");

                break;
        }
    }

    return 0;
}

void ResetBoxDrawing() {

    ui_box_mutex.lock();

    ui_box_done = false;
    ui_box_first_click = false;

    box_top = box_bottom;
    box_bottom = Point2d(-1, -1);

    ui_box_mutex.unlock();
}

/**
 * Mouse callback so that the user can click on an image
 */
void OnMouse( int event, int x, int y, int flags, void* hud_in) {

    Hud *hud = (Hud*) hud_in;

    if (ui_box) {
        ui_box_mutex.lock();

        if( event == EVENT_LBUTTONUP) { // left button click
            if (ui_box_first_click) {
                ui_box_done = true;
            } else {
                ui_box_first_click = true;
            }
        }

        if (ui_box_first_click) {
            box_bottom = Point2d(x / hud->GetImageScaling(), y / hud->GetImageScaling());

        } else {
            box_top = Point2d(x / hud->GetImageScaling(), y / hud->GetImageScaling());
            box_bottom = Point2d(-1, -1);
        }

        ui_box_mutex.unlock();
    }
}

void AskForFrame(int video_number, int frame_number) {
    // construct a replay message
    lcmt_stereo msg;

    msg.timestamp = 0;
    msg.number_of_points = 0;
    msg.video_number = video_number;
    msg.frame_number = frame_number;

    lcmt_stereo_publish(lcm, "stereo_replay", &msg);
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    if (box_file.is_open()) {
        box_file.close();
    }

    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}



void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    stereo_mutex.lock();
    if (last_stereo_msg) {
        delete last_stereo_msg;
    }

    last_stereo_msg = lcmt_stereo_copy(msg);
    stereo_mutex.unlock();
}

void stereo_xy_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_with_xy *msg, void *user) {
    stereo_xy_mutex.lock();
    if (last_stereo_xy_msg) {
        delete last_stereo_xy_msg;
    }

    last_stereo_xy_msg = lcmt_stereo_with_xy_copy(msg);
    stereo_xy_mutex.unlock();
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

    image_mutex.lock();

    if (msg->pixelformat == 1196444237) { // PIXEL_FORMAT_MJPEG

        left_image = Mat::zeros(msg->height, msg->width, CV_8UC1);

        // decompress JPEG
        jpeg_decompress_8u_gray(msg->data, msg->size, left_image.data, msg->width, msg->height, left_image.step);
        //jpeg_decompress_8u_rgb(msg->data, msg->size, left_image.data, msg->width, msg->height, left_image.step);

    } else if (msg->pixelformat == 1497715271) { // PIXEL_FORMAT_GRAY

        Mat temp_image(msg->height, msg->width, CV_8UC1, msg->data);

        temp_image.copyTo(left_image);

    } else if (msg->pixelformat == 859981650) { // PIXEL_FORMAT_RGB

        Mat temp_image(msg->height, msg->width, CV_8UC3, msg->data, msg->row_stride);

        cvtColor(temp_image, left_image, CV_RGB2BGR);

    } else {
        cerr << "Warning: reading images other than GRAY and JPEG not yet implemented." << endl;
        return;
    }


    real_frame_loaded = true;


    image_mutex.unlock();

}

// for replaying videos, subscribe to the stereo replay channel and set the frame number
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    Hud *hud = (Hud*)user;

    hud->SetFrameNumber(msg->frame_number);
    hud->SetVideoNumber(msg->video_number);

    stereo_replay_mutex.lock();
    last_stereo_replay_msg = lcmt_stereo_copy(msg);
    stereo_replay_mutex.unlock();

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

void Get2DPointsFromLcmXY(lcmt_stereo_with_xy *msg, vector<Point> *xy_points) {
    for (int i = 0; i < msg->number_of_points; i ++) {
        xy_points->push_back(Point(msg->frame_x[i], msg->frame_y[i]));
    }
}

void Draw2DPointsOnImage(Mat image, vector<Point> *points) {
    for (Point point : *points) {
        rectangle(image, Point(point.x-1, point.y-1), Point(point.x+1, point.y+1), Scalar(0, 0, 0.8));
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


