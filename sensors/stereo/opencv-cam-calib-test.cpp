#include "opencv-cam-calib-test.hpp"

bool left_camera_on = false;
bool right_camera_on = false;

dc1394_t        *d;
dc1394camera_t  *camera;

dc1394_t        *d2;
dc1394camera_t  *camera2;


int main(int argc, char *argv[]) {

    // --- setup control-c handling ---
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = control_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    // --- end ctrl-c handling code ---

    
    Mat m1_mat, d1_mat, m2_mat, d2_mat;
    
    

    OpenCvStereoConfig stereo_config;
    string config_file = "";
    
    
    ConciseArgs parser(argc, argv);
    parser.add(config_file, "c", "config", "Configuration file containing camera GUIDs, etc.", true);
    parser.add(left_camera_on, "l", "left-camera", "View just the left camera, requires stereo_camera_calibrate/M1 and D1-single.xml to exist");
    parser.add(right_camera_on, "r", "right-camera", "View just the right camera, requires stereo_camera_calibrate/M2 and D2-single.xml to exist");
    parser.parse();
    
    
    // parse the config file
    if (ParseConfigFile(config_file, &stereo_config) != true)
    {
        fprintf(stderr, "Failed to parse configuration file, quitting.\n");
        return -1;
    }


    // read in parameter values
    if (left_camera_on == false && right_camera_on == false) {
        // stereo mode
        left_camera_on = true;
        right_camera_on = true;
    }
    
    if (left_camera_on) {
        printf("Attemping to load camera paramters:\n\tstereo_camera_calibrate/M1-single.xml\n\tstereo_camera_calibrate/D1-single.xml\n");
        
        CvMat *M1p = (CvMat *)cvLoad("stereo_camera_calibrate/M1-single.xml",NULL,NULL,NULL);
        if (M1p == NULL)
        {
            fprintf(stderr, "Error: Failed to load stereo_camera_calibrate/M1-single.xml\n");
            exit(-1);
        } else {
            m1_mat = Mat(M1p, true);
        }
        
        CvMat *D1p = (CvMat *)cvLoad("stereo_camera_calibrate/D1-single.xml",NULL,NULL,NULL);
        if (D1p == NULL)
        {
            fprintf(stderr, "Error: Failed to load stereo_camera_calibrate/D1-single.xml\n");
            exit(-1);
        } else {
            d1_mat = Mat(D1p, true);
        }
    }
    printf("done.\n");
    

    if (right_camera_on) {
        printf("Attemping to load camera paramters:\n\tstereo_camera_calibrate/M2-single.xml\n\tstereo_camera_calibrate/D2-single.xml\n");
    
        CvMat *M2p = (CvMat *)cvLoad("stereo_camera_calibrate/M2-single.xml",NULL,NULL,NULL);
        if (M2p == NULL)
        {
            fprintf(stderr, "Error: Failed to load stereo_camera_calibrate/M2-single.xml\n");
            exit(-1);
        } else {
            m2_mat = Mat(M2p, true);
        }
        
        CvMat *D2p = (CvMat *)cvLoad("stereo_camera_calibrate/D2-single.xml",NULL,NULL,NULL);
        if (D2p == NULL)
        {
            fprintf(stderr, "Error: Failed to load stereo_camera_calibrate/D2-single.xml\n");
            exit(-1);
        } else {
            d2_mat = Mat(D2p, true);
        }
    }
    printf("done.\n");
    
    
    printf("XML files loaded successfully.\n");
    
    
    // ---------- load up the cameras -----------
    
    uint64 guid = stereo_config.guidLeft;
    uint64 guid2 = stereo_config.guidRight;
    
    dc1394error_t   err;
    

    d = dc1394_new ();
    d2 = dc1394_new ();
    
    if (!d)
        g_critical("Could not create dc1394 context");
        
    camera = dc1394_camera_new (d, guid);
    if (!camera)
        g_critical("Could not create dc1394 camera");

    camera2 = dc1394_camera_new (d2, guid2);
    if (!camera2)
        g_critical("Could not create dc1394 camera for camera 2");

    // setup
    err = setup_gray_capture(camera, DC1394_VIDEO_MODE_FORMAT7_1);
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not setup camera");

    err = setup_gray_capture(camera2, DC1394_VIDEO_MODE_FORMAT7_1);
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera2), "Could not setup camera number 2");
    
    // enable auto-exposure
    // turn on the auto exposure feature
    err = dc1394_feature_set_power(camera, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not turn on the exposure feature");
    
    err = dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    DC1394_ERR_RTN(err,"Could not turn on Auto-exposure");
    
    // enable auto-exposure
    // turn on the auto exposure feature
    err = dc1394_feature_set_power(camera2, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not turn on the exposure feature for cam2");
    
    err = dc1394_feature_set_mode(camera2, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    DC1394_ERR_RTN(err,"Could not turn on Auto-exposure for cam2");
    
    // enable camera
    err = dc1394_video_set_transmission(camera, DC1394_ON);
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not start camera iso transmission");
    err = dc1394_video_set_transmission(camera2, DC1394_ON);
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera2), "Could not start camera iso transmission for camera number 2");
    
    // ------------ end load cameras ------------
    
    InitBrightnessSettings(camera, camera2);
    
    // now we've loaded the calibrations, start showing images!
    
    // init display windows
    if (left_camera_on) {
    	namedWindow("Input Left", CV_WINDOW_AUTOSIZE);
    	moveWindow("Input Left", 100, 100);
    	
    	namedWindow("Undistort Left", CV_WINDOW_AUTOSIZE);
    	moveWindow("Undistort Left", 100, 369);
    }
    
    if (right_camera_on) {
    	namedWindow("Input Right", CV_WINDOW_AUTOSIZE);
    	moveWindow("Input Right", 478, 100);
    	
    	namedWindow("Undistort Right", CV_WINDOW_AUTOSIZE);
    	moveWindow("Undistort Right", 478, 369);
    }
    
    MatchBrightnessSettings(camera, camera2, true);
    
    bool done = false;
    
    while (done == false) {
        // grab frames
        
        Mat left, right;
        
        if (left_camera_on) {
            left = GetFrameFormat7(camera);
            
            // use calibration to undistort image
            Mat left_ud;
            undistort(left, left_ud, m1_mat, d1_mat);
            
            // display
            imshow("Input Left", left);
            
            imshow("Undistort Left", left_ud);
        }
        
        if (right_camera_on) {
            right = GetFrameFormat7(camera2);
            // use calibration to undistort image
            Mat right_ud;
            undistort(right, right_ud, m2_mat, d2_mat);
            
            imshow("Input Right", right);
            
            imshow("Undistort Right", right_ud);
        }
        
        char key = waitKey(5);
        
         switch (key) {
            case 'q':
                done = true;
                break;
        }
    }
    
    
    CleanUp();
    
    return 0;
    
}


void CleanUp() {
    if (left_camera_on) {
      
        StopCapture(d, camera);
    }
    
    if (right_camera_on) {
        StopCapture(d2, camera2);
    }
}


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
  
    CleanUp();
    
    
    exit(0);
}
