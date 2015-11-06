/**
 * Program that runs pushbroom stereo.
 * This grabs stereo pairs from Point Grey
 * Firefly MV USB cameras, perfroms the single-disparity
 * stereo algorithm on them, and publishes the results
 * to LCM.
 *
 * Copyright 2013-2015, Andrew Barry <abarry@csail.mit.edu>
 *
 */

#include "pushbroom-stereo-main.hpp"

bool show_display; // set to true to show opencv images on screen
int show_display_wait = 1; // milli seconds to wait between frames when showing display
bool disable_stereo = false;
bool show_unrectified = false;
bool display_hud = false;
bool record_hud = false;
bool visualize_stereo_hits = true;
bool publish_all_images = false;
lcmt_stereo *stereo_lcm_msg = NULL; // for use in visualizing stereo hits recorded on the fly

int force_brightness = -1;
int force_exposure = -1;

int inf_sad_add = 0;
int y_offset = 0;
int file_frame_skip = 0;
int current_video_number = -1;

RecordingManager recording_manager;

// global for where we are drawing a line on the image
int lineLeftImgPosition = -1;
int lineLeftImgPositionY = -1;

// lcm subscription to the control channel
lcmt_stereo_control_subscription_t *stereo_control_sub;

PushbroomStereoState state; // HACK

// subscriptions to data
lcmt_stereo_subscription_t *stereo_replay_sub;
lcmt_cpu_info_subscription_t *cpu_info_sub1;
lcmt_cpu_info_subscription_t *cpu_info_sub2;
lcmt_cpu_info_subscription_t *cpu_info_sub3;
lcmt_log_size_subscription_t *log_size_sub1;
lcmt_log_size_subscription_t *log_size_sub2;
lcmt_log_size_subscription_t *log_size_sub3;
mav_pose_t_subscription_t *mav_pose_t_sub;
lcmt_baro_airspeed_subscription_t *baro_airspeed_sub;
lcmt_battery_status_subscription_t *battery_status_sub;
lcmt_deltawing_u_subscription_t *servo_out_sub;
mav_gps_data_t_subscription_t *mav_gps_data_t_sub;

int numFrames = 0;
bool quiet_mode = false;

double timer_sum = 0;
int timer_count = 0;

dc1394_t        *d;
dc1394camera_t  *camera;

dc1394_t        *d2;
dc1394camera_t  *camera2;

OpenCvStereoConfig stereoConfig;

/**
 * Cleanly handles and exit from a command-line
 * ctrl-c signal.
 *
 * @param s
 *
 */
void control_c_handler(int s)
{
    cout << endl << "exiting via ctrl-c" << endl;

    if (recording_manager.UsingLiveCameras()) {

        StopCapture(d, camera);
        StopCapture(d2, camera2);

        cout << "\tpress ctrl+\\ to quit while writing video." << endl;
        recording_manager.FlushBufferToDisk();
    }

    exit(0);
}


/**
 * Handles LCM control messages.  Is called when an LCM message is received on the channel.
 */
void lcm_stereo_control_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_control *msg, void *user)
{
    /*
     * The way this works is that as soon as
     * the program starts, stereo is always running.
     *
     * stereo control:
     *   0: write to disk and then restart
     *   1: (re)start record
     *   2: pause
     *
     */

    // got a control message, so parse it and figure out what we should do
    if (msg->stereo_control == 0)
    {
        // write video, then start recording again

        recording_manager.FlushBufferToDisk();
        recording_manager.BeginNewRecording();

    } else if (msg->stereo_control == 1)
    {
        // record
        cout << endl << "(Re)starting recording." << endl;


        recording_manager.BeginNewRecording();

    } else if (msg->stereo_control == 2)
    {
        // pause recording if we were recording
        cout << "Recording paused." << endl;

        recording_manager.SetRecordingOn(false);

    } else {
        // got an unknown command
        fprintf(stderr, "WARNING: Unknown stereo_control command: %d", int(msg->stereo_control));
    }

}


int main(int argc, char *argv[])
{
    // get input arguments

    string configFile = "";
    string video_file_left = "", video_file_right = "", video_directory = "";
    int starting_frame_number = 0;
    bool enable_gamma = false;
    float random_results = -1.0;

    int last_frame_number = -1;

    int last_playback_frame_number = -2;

    ConciseArgs parser(argc, argv);
    parser.add(configFile, "c", "config", "Configuration file containing camera GUIDs, etc.", true);
    parser.add(show_display, "d", "show-dispaly", "Enable for visual debugging display. Will reduce framerate significantly.");
    parser.add(show_display_wait, "w", "show-display-wait", "Optional argument to decrease framerate for lower network traffic when forwarding the display.");
    parser.add(show_unrectified, "u", "show-unrectified", "When displaying images, do not apply rectification.");
    parser.add(disable_stereo, "s", "disable-stereo", "Disable online stereo processing.");
    parser.add(force_brightness, "b", "force-brightness", "Force a brightness setting.");
    parser.add(force_exposure, "e", "force-exposure", "Force an exposure setting.");
    parser.add(quiet_mode, "q", "quiet", "Reduce text output.");
    parser.add(video_file_left, "l", "video-file-left", "Do not use cameras, instead use this video file (also requires a right video file).");
    parser.add(video_file_right, "t", "video-file-right", "Right video file, only for use with the -l option.");
    parser.add(video_directory, "i", "video-directory", "Directory to search for videos in (for playback).");
    parser.add(starting_frame_number, "f", "starting-frame", "Frame to start at when playing back videos.");
    parser.add(display_hud, "v", "hud", "Overlay HUD on display images.");
    parser.add(record_hud, "x", "record-hud", "Record the HUD display.");
    parser.add(file_frame_skip, "p", "skip", "Number of frames skipped in recording (for playback).");
    parser.add(enable_gamma, "g", "enable-gamma", "Turn gamma on for both cameras.");
    parser.add(random_results, "R", "random-results", "Number of random points to produce per frame.  Can be a float in which case we'll take a random sample to decide if to produce the last one.  Disables real stereo processing.  Only for debugging / analysis!");
    parser.add(publish_all_images, "P", "publish-all-images", "Publish all images to LCM");
    parser.parse();

    // parse the config file
    if (ParseConfigFile(configFile, &stereoConfig) != true)
    {
        fprintf(stderr, "Failed to parse configuration file, quitting.\n");
        return -1;
    }

    if (video_file_left.length() > 0
        && video_file_right.length() <= 0) {

        fprintf(stderr, "Error: for playback you must specify both "
            "a right and left video file. (Only got a left one.)\n");

        return -1;
    }

     if (video_file_left.length() <= 0
        && video_file_right.length() > 0) {

        fprintf(stderr, "Error: for playback you must specify both "
            "a right and left video file. (Only got a right one.)\n");

        return -1;
    }

    recording_manager.Init(stereoConfig);

    // attempt to load video files / directories
    if (video_file_left.length() > 0) {
        if (recording_manager.LoadVideoFiles(video_file_left, video_file_right) != true) {
            // don't have videos, bail out.
            return -1;
        }
    }

    if (video_directory.length() > 0) {
        if (recording_manager.SetPlaybackVideoDirectory(video_directory) != true) {
            // bail
            return -1;
        }
    }

    recording_manager.SetQuietMode(quiet_mode);
    recording_manager.SetPlaybackFrameNumber(starting_frame_number);



    uint64 guid = stereoConfig.guidLeft;
    uint64 guid2 = stereoConfig.guidRight;

    // start up LCM
    lcm_t * lcm;
    lcm = lcm_create (stereoConfig.lcmUrl.c_str());


    unsigned long elapsed;

    Hud hud;


    // --- setup control-c handling ---
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = control_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    // --- end ctrl-c handling code ---

    dc1394error_t   err;
    dc1394error_t   err2;


    // tell opencv to use only one core so that we can manage our
    // own threading without a fight
    setNumThreads(1);

    if (recording_manager.UsingLiveCameras()) {
        d = dc1394_new ();
        if (!d)
            cerr << "Could not create dc1394 context" << endl;

        d2 = dc1394_new ();
        if (!d2)
            cerr << "Could not create dc1394 context for camera 2" << endl;

        camera = dc1394_camera_new (d, guid);
        if (!camera)
        {
            cerr << "Could not create dc1394 camera... quitting." << endl;
            exit(1);
        }

        camera2 = dc1394_camera_new (d2, guid2);
        if (!camera2)
            cerr << "Could not create dc1394 camera for camera 2" << endl;
        // reset the bus
        dc1394_reset_bus(camera);
        dc1394_reset_bus(camera2);

        // setup
        err = setup_gray_capture(camera, DC1394_VIDEO_MODE_FORMAT7_1);
        DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not setup camera");

        err2 = setup_gray_capture(camera2, DC1394_VIDEO_MODE_FORMAT7_1);
        DC1394_ERR_CLN_RTN(err2, cleanup_and_exit(camera2), "Could not setup camera number 2");

        // enable camera
        err = dc1394_video_set_transmission(camera, DC1394_ON);
        DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not start camera iso transmission");
        err2 = dc1394_video_set_transmission(camera2, DC1394_ON);
        DC1394_ERR_CLN_RTN(err2, cleanup_and_exit(camera2), "Could not start camera iso transmission for camera number 2");

        InitBrightnessSettings(camera, camera2, enable_gamma);
    }

    if (show_display) {

        namedWindow("Input", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
        namedWindow("Input2", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
        namedWindow("Stereo", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);

        namedWindow("Left Block", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
        namedWindow("Right Block", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);

        namedWindow("Debug 1", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
        namedWindow("Debug 2", CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);



        setMouseCallback("Input", onMouse); // for drawing disparity lines
        setMouseCallback("Stereo", onMouseStereo, &hud); // for drawing disparity lines

        moveWindow("Input", stereoConfig.displayOffsetX + 100, stereoConfig.displayOffsetY + 100);
        moveWindow("Stereo", stereoConfig.displayOffsetX + 100, stereoConfig.displayOffsetY + 370);
        moveWindow("Input2", stereoConfig.displayOffsetX + 478, stereoConfig.displayOffsetY + 100);
        moveWindow("Left Block", stereoConfig.displayOffsetX + 900, stereoConfig.displayOffsetY + 100);
        moveWindow("Right Block", stereoConfig.displayOffsetX + 1400, stereoConfig.displayOffsetY + 100);

        moveWindow("Debug 1", stereoConfig.displayOffsetX + 900, stereoConfig.displayOffsetY + 670);
        moveWindow("Debug 2", stereoConfig.displayOffsetX + 1400, stereoConfig.displayOffsetY + 670);

    } // show display

    if (show_display || publish_all_images) {
        // if a channel exists, subscribe to it
        if (stereoConfig.stereo_replay_channel.length() > 0) {
            stereo_replay_sub = lcmt_stereo_subscribe(lcm, stereoConfig.stereo_replay_channel.c_str(), &stereo_replay_handler, &hud);
        }

        if (stereoConfig.pose_channel.length() > 0) {
            mav_pose_t_sub = mav_pose_t_subscribe(lcm, stereoConfig.pose_channel.c_str(), &mav_pose_t_handler, &hud);
        }

        if (stereoConfig.gps_channel.length() > 0) {
            mav_gps_data_t_sub = mav_gps_data_t_subscribe(lcm, stereoConfig.gps_channel.c_str(), &mav_gps_data_t_handler, &hud);
        }

        if (stereoConfig.baro_airspeed_channel.length() > 0) {
            baro_airspeed_sub = lcmt_baro_airspeed_subscribe(lcm, stereoConfig.baro_airspeed_channel.c_str(), &baro_airspeed_handler, &hud);
        }

        if (stereoConfig.servo_out_channel.length() > 0) {
            servo_out_sub = lcmt_deltawing_u_subscribe(lcm, stereoConfig.servo_out_channel.c_str(), &servo_out_handler, &hud);
        }

        if (stereoConfig.battery_status_channel.length() > 0) {
            battery_status_sub = lcmt_battery_status_subscribe(lcm, stereoConfig.battery_status_channel.c_str(), &battery_status_handler, &hud);
        }

        if (stereoConfig.cpu_info_channel1.length() > 0) {
            cpu_info_sub1 = lcmt_cpu_info_subscribe(lcm, stereoConfig.cpu_info_channel1.c_str(), &cpu_info_handler, &recording_manager);
            cpu_info_sub2 = lcmt_cpu_info_subscribe(lcm, stereoConfig.cpu_info_channel2.c_str(), &cpu_info_handler, &recording_manager);
            cpu_info_sub3 = lcmt_cpu_info_subscribe(lcm, stereoConfig.cpu_info_channel3.c_str(), &cpu_info_handler, &recording_manager);
        }

        if (stereoConfig.log_size_channel1.length() > 0) {
            log_size_sub1 = lcmt_log_size_subscribe(lcm, stereoConfig.log_size_channel1.c_str(), &log_size_handler, &hud);
            log_size_sub2 = lcmt_log_size_subscribe(lcm, stereoConfig.log_size_channel2.c_str(), &log_size_handler, &hud);
            log_size_sub3 = lcmt_log_size_subscribe(lcm, stereoConfig.log_size_channel3.c_str(), &log_size_handler, &hud);
        }

    } // end show_display || publish_all_images

    // load calibration
    OpenCvStereoCalibration stereoCalibration;

    if (LoadCalibration(stereoConfig.calibrationDir, &stereoCalibration) != true)
    {
        cerr << "Error: failed to read calibration files. Quitting." << endl;
        return -1;
    }

    int inf_disparity_tester, disparity_tester;
    disparity_tester = GetDisparityForDistance(10, stereoCalibration, &inf_disparity_tester);

    std::cout << "computed disparity is = " << disparity_tester << ", inf disparity = " << inf_disparity_tester << std::endl;

    // subscribe to the stereo control channel
    stereo_control_sub = lcmt_stereo_control_subscribe(lcm, stereoConfig.stereoControlChannel.c_str(), &lcm_stereo_control_handler, NULL);


    Mat imgDisp;
    Mat imgDisp2;

    // initilize default parameters
    //PushbroomStereoState state; // HACK

    state.disparity = stereoConfig.disparity;
    state.zero_dist_disparity = stereoConfig.infiniteDisparity;
    state.sobelLimit = stereoConfig.interestOperatorLimit;
    state.horizontalInvarianceMultiplier = stereoConfig.horizontalInvarianceMultiplier;
    state.blockSize = stereoConfig.blockSize;
    state.random_results = random_results;
    state.check_horizontal_invariance = true;

    if (state.blockSize > 10 || state.blockSize < 1)
    {
        fprintf(stderr, "Warning: block size is very large "
            "or small (%d).  Expect trouble.\n", state.blockSize);
    }

    state.sadThreshold = stereoConfig.sadThreshold;

    state.mapxL = stereoCalibration.mx1fp;
    state.mapxR = stereoCalibration.mx2fp;
    state.Q = stereoCalibration.qMat;
    state.show_display = show_display;

    state.lastValidPixelRow = stereoConfig.lastValidPixelRow;

    Mat matL, matR;
    bool quit = false;

    if (recording_manager.UsingLiveCameras()) {
        matL = GetFrameFormat7(camera);
        matR = GetFrameFormat7(camera2);

        if (recording_manager.InitRecording(matL, matR) != true) {
            // failed to init recording, things are going bad.  bail.
            return -1;
        }

        // before we start, turn the cameras on and set the brightness and exposure
        MatchBrightnessSettings(camera, camera2, true, force_brightness, force_exposure);

        // grab a few frames and send them over LCM for the user
        // to verify that everything is working
        if (!show_display && !publish_all_images) {
            printf("Sending init images over LCM... ");
            fflush(stdout);

            for (int i = 0; i < 5; i++) {

                matL = GetFrameFormat7(camera);
                SendImageOverLcm(lcm, "stereo_image_left", matL, 50);

                matR = GetFrameFormat7(camera2);
                SendImageOverLcm(lcm, "stereo_image_right", matR, 50);

                // don't send these too fast, otherwise we'll flood the ethernet link
                // and not actually be helpful

                // wait one second
                printf(".");
                fflush(stdout);

                sleep(1);
            }
            printf(" done.\n");
        }

    } // recording_manager.UsingLiveCameras()

    // spool up worker threads
    PushbroomStereo pushbroom_stereo;

    // start the framerate clock
    struct timeval start, now;
    gettimeofday( &start, NULL );

    while (quit == false) {

        // get the frames from the camera
        if (recording_manager.UsingLiveCameras()) {
            // we would like to match brightness every frame
            // but that would really hurt our framerate
            // match brightness every 10 frames instead
            if (numFrames % MATCH_BRIGHTNESS_EVERY_N_FRAMES == 0)
            {
                MatchBrightnessSettings(camera, camera2);
            }

            // capture images from the cameras
            matL = GetFrameFormat7(camera);
            matR = GetFrameFormat7(camera2);

            // record video
            recording_manager.AddFrames(matL, matR);


        } else {
            // using a video file -- get the next frame
            recording_manager.GetFrames(matL, matR);
        }

        cv::vector<Point3f> pointVector3d;
        cv::vector<uchar> pointColors;
        cv::vector<Point3i> pointVector2d; // for display
        cv::vector<Point3i> pointVector2d_inf; // for display

        // do the main stereo processing
        if (disable_stereo != true) {

            gettimeofday( &now, NULL );
            double before = now.tv_usec + now.tv_sec * 1000 * 1000;

            pushbroom_stereo.ProcessImages(matL, matR, &pointVector3d, &pointColors, &pointVector2d, state);

            gettimeofday( &now, NULL );
            double after = now.tv_usec + now.tv_sec * 1000 * 1000;

            timer_sum += after-before;
            timer_count ++;

        }

        // build an LCM message for the stereo data
        lcmt_stereo msg;


        if (recording_manager.UsingLiveCameras() || stereo_lcm_msg == NULL) {
            msg.timestamp = getTimestampNow();
        } else {
            // if we are replaying videos, preserve the timestamp of the original video
            msg.timestamp = stereo_lcm_msg->timestamp;

        }


        msg.number_of_points = (int)pointVector3d.size();

        float x[msg.number_of_points];
        float y[msg.number_of_points];
        float z[msg.number_of_points];
        uchar grey[msg.number_of_points];

        for (unsigned int i=0;i<pointVector3d.size();i++) {

            x[i] = pointVector3d[i].x / stereoConfig.calibrationUnitConversion;
            y[i] = pointVector3d[i].y / stereoConfig.calibrationUnitConversion;
            z[i] = pointVector3d[i].z / stereoConfig.calibrationUnitConversion;
            grey[i] = pointColors[i];
        }

        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.grey = grey;
        msg.frame_number = recording_manager.GetFrameNumber();

        if (recording_manager.UsingLiveCameras()) {
            msg.frame_number = msg.frame_number - 1;  // minus one since recording manager has
                                                      // already recorded this frame (above in
                                                      // AddFrames) but we haven't made a message
                                                      // for it yet
        }


        msg.video_number = recording_manager.GetRecVideoNumber();

        // publish the LCM message
        if (last_frame_number != msg.frame_number) {
            lcmt_stereo_publish(lcm, "stereo", &msg);
            last_frame_number = msg.frame_number;
        }

        if (publish_all_images) {
            if (recording_manager.GetFrameNumber() != last_playback_frame_number) {
                SendImageOverLcm(lcm, "stereo_image_left", matL, 80);
                SendImageOverLcm(lcm, "stereo_image_right", matR, 80);

                last_playback_frame_number = recording_manager.GetFrameNumber();
            }

            //process LCM until there are no more messages
            // this allows us to drop frames if we are behind
            while (NonBlockingLcm(lcm)) {}
        }

        Mat matDisp, remapL, remapR;

        if (show_display) {
            // we remap again here because we're just in display
            Mat remapLtemp(matL.rows, matL.cols, matL.depth());
            Mat remapRtemp(matR.rows, matR.cols, matR.depth());

            remapL = remapLtemp;
            remapR = remapRtemp;

            remap(matL, remapL, stereoCalibration.mx1fp, Mat(), INTER_NEAREST);
            remap(matR, remapR, stereoCalibration.mx2fp, Mat(), INTER_NEAREST);

            remapL.copyTo(matDisp);

            //process LCM until there are no more messages
            // this allows us to drop frames if we are behind
            while (NonBlockingLcm(lcm)) {}
        } // end show_display


        if (show_display) {

            for (unsigned int i=0;i<pointVector2d.size();i++) {
                int x2 = pointVector2d[i].x;
                int y2 = pointVector2d[i].y;
                //int sad = pointVector2d[i].z;
                rectangle(matDisp, Point(x2,y2), Point(x2+state.blockSize, y2+state.blockSize), 0,  CV_FILLED);
                rectangle(matDisp, Point(x2+1,y2+1), Point(x2+state.blockSize-1, y2-1+state.blockSize), 255);

            }

            // draw pixel blocks
            if (lineLeftImgPosition >= 0 && lineLeftImgPositionY > 1) {
                DisplayPixelBlocks(remapL, remapR, lineLeftImgPosition - state.blockSize/2, lineLeftImgPositionY - state.blockSize/2, state, &pushbroom_stereo);
            }

            // draw a line for the user to show disparity
            DrawLines(remapL, remapR, matDisp, lineLeftImgPosition, lineLeftImgPositionY, state.disparity, state.zero_dist_disparity);


            if (visualize_stereo_hits == true && stereo_lcm_msg != NULL) {

                // transform the points from 3D space back onto the image's 2D space
                vector<Point3f> lcm_points;
                Get3DPointsFromStereoMsg(stereo_lcm_msg, &lcm_points);

                // draw the points on the unrectified image (to see these
                // you must pass the -u flag)
                Draw3DPointsOnImage(matL, &lcm_points, stereoCalibration.M1, stereoCalibration.D1, stereoCalibration.R1, 128);

            }

            if (show_unrectified == false) {

                imshow("Input", remapL);
                imshow("Input2", remapR);
            } else {
                imshow("Input", matL);
                imshow("Input2", matR);
            }


            if (display_hud) {
                Mat with_hud;

                recording_manager.SetHudNumbers(&hud);

                hud.DrawHud(matDisp, with_hud);

                if (record_hud) {
                    // put this frame into the HUD recording
                    recording_manager.RecFrameHud(with_hud);

                }

                imshow("Stereo", with_hud);
            } else {
                imshow("Stereo", matDisp);
            }


            char key = waitKey(show_display_wait);

            if (key != 255 && key != -1)
            {
                cout << endl << key << endl;
            }

            switch (key)
            {
                case 'T':
                    state.disparity --;
                    break;
                case 'R':
                    state.disparity ++;
                    break;

                case 'w':
                    state.sobelLimit += 10;
                    break;

                case 's':
                    state.sobelLimit -= 10;
                    break;

                case 'd':
                    state.horizontalInvarianceMultiplier -= 0.1;
                    break;

                case 'D':
                    state.horizontalInvarianceMultiplier += 0.1;
                    break;

                case 'g':
                    state.blockSize ++;
                    break;

                case 'b':
                    state.blockSize --;
                    if (state.blockSize < 1) {
                        state.blockSize = 1;
                    }
                    break;

                case 'Y':
                    state.sadThreshold += 50;
                    break;

                case 'y':
                    state.sadThreshold ++;
                    break;

                case 'h':
                    state.sadThreshold --;
                    break;

                case 'H':
                    state.sadThreshold -= 50;
                    break;

                case 'm':
                    if (recording_manager.UsingLiveCameras()) {
                        MatchBrightnessSettings(camera, camera2, true, force_brightness, force_exposure);
                    }
                    break;

                case '1':
                    force_brightness --;
                    if (recording_manager.UsingLiveCameras()) {
                        MatchBrightnessSettings(camera, camera2, true, force_brightness, force_exposure);
                    }
                    break;

                case '2':
                    force_brightness ++;
                    if (recording_manager.UsingLiveCameras()) {
                        MatchBrightnessSettings(camera, camera2, true, force_brightness, force_exposure);
                    }
                    break;

                case '3':
                    force_exposure --;
                    if (recording_manager.UsingLiveCameras()) {
                        MatchBrightnessSettings(camera, camera2, true, force_brightness, force_exposure);
                    }
                    break;

                case '4':
                    force_exposure ++;
                    if (recording_manager.UsingLiveCameras()) {
                        MatchBrightnessSettings(camera, camera2, true, force_brightness, force_exposure);
                    }
                    break;

                case '5':
                    // to show SAD boxes
                    state.sobelLimit = 0;
                    state.sadThreshold = 255;
                    break;

                case 'I':
                    state.check_horizontal_invariance = !state.check_horizontal_invariance;
                    break;

                case '.':
                    recording_manager.SetPlaybackFrameNumber(recording_manager.GetFrameNumber() + 1);
                    break;

                case ',':
                    recording_manager.SetPlaybackFrameNumber(recording_manager.GetFrameNumber() - 1);
                    break;

                case '>':
                    recording_manager.SetPlaybackFrameNumber(recording_manager.GetFrameNumber() + 50);
                    break;

                case '<':
                    recording_manager.SetPlaybackFrameNumber(recording_manager.GetFrameNumber() - 50);
                    break;

                //case 'k':
                //    state.zero_dist_disparity ++;
                 //   break;

                case 'l':
                    state.zero_dist_disparity --;
                    break;

                case 'o':
                    inf_sad_add --;
                    break;

                case 'p':
                    inf_sad_add ++;
                    break;

                case '[':
                    y_offset --;
                    if (y_offset < 0) {
                        y_offset = 0;
                    }
                    break;

                case ']':
                    y_offset ++;
                    break;

                case 'v':
                    display_hud = !display_hud;
                    break;

                case 'c':
                    hud.SetClutterLevel(hud.GetClutterLevel() + 1);
                    break;

                case 'C':
                    hud.SetClutterLevel(hud.GetClutterLevel() - 1);
                    break;

                case '}':
                    hud.SetPitchRangeOfLens(hud.GetPitchRangeOfLens() + 1);
                    break;
                case '{':
                    hud.SetPitchRangeOfLens(hud.GetPitchRangeOfLens() - 1);
                    break;

                case 'S':
                    // take a screen cap of the left and right images
                    // useful for putting into a stereo tuner
                    printf("\nWriting left.ppm...");
                    imwrite("left.ppm", remapL);

                    printf("\nWriting right.ppm...");
                    imwrite("right.ppm", remapR);

                    printf("\ndone.");
                    break;

                case 'V':
                    // record the HUD
                    record_hud = true;
                    recording_manager.RestartRecHud();
                    break;

                    /*
                case 'j':
                    state.debugJ --;
                    break;

                case 'J':
                    state.debugJ ++;
                    break;

                case 'i':
                    state.debugI --;
                    break;

                case 'I':
                    state.debugI ++;
                    break;

                case 'k':
                    state.debugDisparity --;
                    break;

                case 'K':
                    state.debugDisparity ++;
                    break;

                    */

                case 'q':
                    quit = true;
                    break;
            }

            if (key != 255 && key != -1)
            {
                cout << "sadThreshold = " << state.sadThreshold << endl;
                cout << "sobelLimit = " << state.sobelLimit << endl;
                cout << "horizontalInvarianceMultiplier = " << state.horizontalInvarianceMultiplier << endl;
                cout << "brightness: " << force_brightness << endl;
                cout << "exposure: " << force_exposure << endl;
                cout << "disparity = " << state.disparity << endl;
                cout << "inf_disparity = " << state.zero_dist_disparity << endl;
                cout << "inf_sad_add = " << inf_sad_add << endl;
                cout << "blockSize = " << state.blockSize << endl;
                cout << "frame_number = " << recording_manager.GetFrameNumber() << endl;
                cout << "y offset = " << y_offset << endl;
                cout << "PitchRangeOfLens = " << hud.GetPitchRangeOfLens() << endl;
            }
        } // end show_display

        numFrames ++;

        // check for new LCM messages
        NonBlockingLcm(lcm);

        if (quiet_mode == false || numFrames % 100 == 0) {
            // compute framerate
            gettimeofday( &now, NULL );

            elapsed = (now.tv_usec / 1000 + now.tv_sec * 1000) -
            (start.tv_usec / 1000 + start.tv_sec * 1000);

            printf("\r%d frames (%lu ms) - %4.1f fps | %4.1f ms/frame, stereo: %f", numFrames, elapsed, (float)numFrames/elapsed * 1000, elapsed/(float)numFrames, timer_sum/(double)timer_count);
            fflush(stdout);
        }


    } // end main while loop

    printf("\n\n");

    destroyWindow("Input");
    destroyWindow("Input2");
    destroyWindow("Stereo");

    // close camera
    if (recording_manager.UsingLiveCameras()) {
        StopCapture(d, camera);
        StopCapture(d2, camera2);
    }

    return 0;
}


/**
 * Mouse callback so that the user can click on an image
 * and see where the disparity line is on the other image pair
 */
void onMouse( int event, int x, int y, int flags, void* )
{
    if( flags & CV_EVENT_FLAG_LBUTTON)
    {
        // paint a line on the image they clicked on
        lineLeftImgPosition = x;
        lineLeftImgPositionY = y;
        cout << x << ", " << y << endl;
    }
}

/**
 * Mouse callback for the stereo image so that the user can click on an image
 * and see where the disparity line is on the other image pair
 */
void onMouseStereo( int event, int x, int y, int flags, void* hud) {
    Hud *hud_in = (Hud*) hud;

    if( flags & CV_EVENT_FLAG_LBUTTON)
    {
        state.debugI = y;
        state.debugJ = x;

        // paint a line on the image they clicked on
        if (display_hud) {
            // check for scaling on the hud
            lineLeftImgPosition = x / hud_in->GetImageScaling();
            lineLeftImgPositionY = y / hud_in->GetImageScaling();
        } else {
            lineLeftImgPosition = x;
            lineLeftImgPositionY = y;
        }
    }
}

/**
 * Draws lines on the images for stereo debugging.
 *
 * @param rightImg right image
 * @param stereoImg stereo image
 * @param lineX x position of the line to draw
 * @param lineY y position of the line to draw
 * @param disparity disparity move the line on the right image over by
 * @param inf_disparity disparity corresponding to "infinite distance"
 *  used to filter out false-positives.  Usually availible in
 *   state.zero_dist_disparity.
 */
void DrawLines(Mat leftImg, Mat rightImg, Mat stereoImg, int lineX, int lineY, int disparity, int inf_disparity) {
    int lineColor = 128;
    if (lineX >= 0)
    {
        // print out the values of the pixels where they clicked
        //cout << endl << endl << "Left px: " << (int)leftImg.at<uchar>(lineY, lineX)
        //    << "\tRight px: " << (int)rightImg.at<uchar>(lineY, lineX + disparity)
        //    << endl;

        line(leftImg, Point(lineX, 0), Point(lineX, leftImg.rows), lineColor);
        line(stereoImg, Point(lineX, 0), Point(lineX, leftImg.rows), lineColor);
        line(rightImg, Point(lineX + disparity, 0), Point(lineX + disparity, rightImg.rows), lineColor);

        line(rightImg, Point(lineX + inf_disparity, 0), Point(lineX + inf_disparity, rightImg.rows), lineColor);

        line(leftImg, Point(0, lineY), Point(leftImg.cols, lineY), lineColor);
        line(stereoImg, Point(0, lineY), Point(leftImg.cols, lineY), lineColor);
        line(rightImg, Point(0, lineY), Point(rightImg.cols, lineY), lineColor);
    }
}

/**
 * Writes a disparity map in the style of a normal stereo vision system
 *
 * @param pointVector2d output from stereo algorithm
 * @param state PushbroomStereoState used to process stereo
 * @param pixel_value value to fill in this round of pixels, from 0 to 255
 *  @default 128
 * @param existing_map provide this if you want to write on top of an existing
 *  image.  Must be 376x240, CV_8UC1.
 *  @default Mat::zeros(240, 376, CV_8UC1);
 *
 * @retval a Mat that contains the single disparity map.  It is CV_8UC1 and
 *      sized 376x240.
 */
Mat WriteDisparityMap(cv::vector<Point3i> *pointVector2d, PushbroomStereoState state, int pixel_value, Mat existing_map) {

    // init the mat as all black

    //Mat disp = Mat::zeros(240, 376, CV_8UC1);


    // now write squares based on block size and location

    for (int i=0; i<(int)pointVector2d->size(); i++) {

        // for each spot, fill a box of the right size

        int x = pointVector2d->at(i).x;
        int y = pointVector2d->at(i).y;

        rectangle(existing_map, Point(x, y), Point(x + state.blockSize, y + state.blockSize), pixel_value, CV_FILLED);

    }

    return existing_map;
}



/**
 * Displays a very zoomed in version of the two pixel blocks being looked at
 *
 * @param left_image the left image
 * @param right_image the right image
 * @param left left coordinate of the box
 * @param top top coordinate of the box
 * @param state PushbroomStereoState containing stereo information
 * @param pushbroom_stereo stereo object so we can run GetSAD
 *
 */
void DisplayPixelBlocks(Mat left_image, Mat right_image, int left, int top, PushbroomStereoState state, PushbroomStereo *pushbroom_stereo) {
    if (left + state.blockSize > left_image.cols || top+state.blockSize > left_image.rows
        || left + state.blockSize > right_image.cols || top+state.blockSize > right_image.rows
        || left + state.disparity < 0) { // remember, disparity can be negative

        // invalid spot

        return;
    }

    Mat left_block = left_image.rowRange(top, top+state.blockSize).colRange(left, left+state.blockSize);
    Mat right_block = right_image.rowRange(top, top+state.blockSize).colRange(left+state.disparity, left+state.disparity+state.blockSize);


    Mat laplacian_left;
    Laplacian(left_image, laplacian_left, -1, 3, 1, 0, BORDER_DEFAULT);

    Mat laplacian_right;
    Laplacian(right_image, laplacian_right, -1, 3, 1, 0, BORDER_DEFAULT);

    // compute stats about block
    int left_interest, right_interest, raw_sad;
    int sad = pushbroom_stereo->GetSAD(left_image, right_image, laplacian_left, laplacian_right, left, top,
        state, &left_interest, &right_interest, &raw_sad);

    float diff_score = 100*(float)abs(left_interest - right_interest)/(float)(left_interest+right_interest);

    // make the blocks visible by making them huge
    const int scale_factor = 100;

    Size output_size = Size(state.blockSize * scale_factor, state.blockSize * scale_factor);
    resize(left_block, left_block, output_size, 0, 0, INTER_NEAREST);
    resize(right_block, right_block, output_size, 0, 0, INTER_NEAREST);

    char sad_str_char[100], left_interest_char[100], right_interest_char[100],
        diff_score_char[100], raw_sad_char[100];
    sprintf(sad_str_char, "SAD = %d", sad);
    sprintf(left_interest_char, "Left interest = %d", left_interest);
    sprintf(right_interest_char, "Right interest = %d", right_interest);
    sprintf(raw_sad_char, "Raw sad = %d", raw_sad);
    sprintf(diff_score_char, "Diff score = %.02f", diff_score);

    putText(right_block, raw_sad_char, Point(310, 425), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, sad_str_char, Point(400, 445), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, left_interest_char, Point(310, 465), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, right_interest_char, Point(305, 485), FONT_HERSHEY_PLAIN, 1, 0);
    putText(right_block, diff_score_char, Point(305, 495), FONT_HERSHEY_PLAIN, 1, 0);

    imshow("Left Block", left_block);
    imshow("Right Block", right_block);



}

// for replaying videos, subscribe to the stereo replay channel and set the frame
// number
void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    recording_manager.SetPlaybackVideoNumber(msg->video_number, msg->timestamp);
    recording_manager.SetPlaybackFrameNumber(msg->frame_number);

    // in addition to displaying the right frame, optionally visualize the hits recorded during flight

    // if there is an old stereo message, delete it so we don't leak memory
    if (stereo_lcm_msg) {
        delete stereo_lcm_msg;
    }

    // record the message for use later
    stereo_lcm_msg = lcmt_stereo_copy(msg);
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

void log_size_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_log_size *msg, void *user) {
    Hud *hud = (Hud*)user;

    // plane number is the last character of the channel name
    char last_char = channel[strlen(channel)-1];
    int plane_number = last_char - '0'; // this converts a single character to an integer
                                        // and is standards conforming C

    hud->SetPlaneNumber(plane_number);
    hud->SetLogNumber(msg->log_number);
}

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user) {
    Hud *hud = (Hud*)user;

    hud->SetAltitude(msg->pos[2]);
    hud->SetOrientation(msg->orientation[0], msg->orientation[1], msg->orientation[2], msg->orientation[3]);
    hud->SetAcceleration(msg->accel[0], msg->accel[1], msg->accel[2]);
    hud->SetAirspeed(msg->vel[0]);

    hud->SetTimestamp(msg->utime);
}

void cpu_info_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_cpu_info *msg, void *user) {
    RecordingManager *rec_m = (RecordingManager*) user;

    // we just use this to figure out the hostname of the cam computer for video replay

    std::string channel_name(channel);

    std::string hostname = "odroid-cam";
    hostname.push_back(channel_name.back());

    rec_m->SetHostname(hostname);
}


# if 0
/**
 * Function that finds and resets all connected USB Point Grey cameras.
 *
 * @retval true on successful reset, false on failure.
 */
bool ResetPointGreyCameras()
{
    libusb_context *s_libucontext;

    // open the libusb context
    libusb_init(&s_libucontext);

    libusb_device **list;

    ssize_t ret;

    ret = libusb_get_device_list (s_libucontext, &list);

    if (ret == LIBUSB_ERROR_NO_MEM || ret < 1)
    {
        printf("Failed to find device in the libusb context.\n");
        libusb_free_device_list(list, 1);
        libusb_exit(s_libucontext);
        return false;
    }

    int j;
    int libusb_flag = 0;

    for (j=0; j<ret; j++)
    {
        libusb_device *this_device = list[j];
        struct libusb_device_descriptor desc;

        if (libusb_get_device_descriptor(this_device, &desc) == 0)
        {
            // before attempting to open the device and get it's description, look at the vendor and product IDs
            // this also means that we won't attempt to open a bunch of devices we probably don't have permissions to open
            // furthermore, if the user hasn't set up permissions correctly, libusb will print out an error about the permissions
            // giving a good hint at what to do

            cout << desc.idVendor << endl;

            if (desc.idVendor == POINT_GREY_VENDOR_ID && desc.idProduct == POINT_GREY_PRODUCT_ID)
            {
                cout << "found a hit" << endl;

                libusb_device_handle *udev;
                if (libusb_open(this_device, &udev) == 0)
                {
                    cout << "resetting device." << endl;
                    // reset this device
                    cout << libusb_reset_device(udev);
                    cout << "success." << endl;
                    //libusb_close(udev);
                    libusb_flag = 1;
                } else
                {
                    cout << "failed to open device" << endl;
                }
            }
        }
    }

    // cleanup libusb list
    libusb_free_device_list(list, 1);

    libusb_exit(s_libucontext);

    if (libusb_flag == 0)
    {
        printf("Failed to reset the USB cameras.\n");
        return false;
    }

    return true;

}
#endif
