/*
 * Utility functions for opencv-stereo
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */

#include "opencv-stereo-util.hpp"

/**
 * Gets a Format7 frame from a Firefly MV USB camera.
 * The frame will be CV_8UC1 and black and white.
 *
 * @param camera the camera
 *
 * @retval frame as an OpenCV Mat
 */
Mat GetFrameFormat7(dc1394camera_t *camera)
{
    // get a frame
    dc1394error_t err;
    dc1394video_frame_t *frame;

    err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
    DC1394_WRN(err,"Could not capture a frame");

    if (err != 0) {
        std::cout << "Warning: failed to capture a frame, returning black frame." << std::endl;

        // attempt release the buffer
        if (frame) {
            err = dc1394_capture_enqueue(camera, frame);
            DC1394_WRN(err,"releasing buffer after failure");
        }

        // we're totally done, we don't have a frame
        // maybe USB disconnect? Anyway, return a black
        // frame so we won't crash everything else
        // and pray that things will get better soon
        return Mat::zeros(240, 376, CV_8UC1);
    }

    // make a Mat of the right size and type that we attach the the existing data
    Mat matTmp = Mat(frame->size[1], frame->size[0], CV_8UC1, frame->image, frame->size[0]);

    // copy the data out of the ringbuffer so we can give the buffer back
    Mat matOut = matTmp.clone();

    // release the buffer
    err = dc1394_capture_enqueue(camera, frame);
    DC1394_WRN(err,"releasing buffer");

    return matOut;
}

/**
 * Flushes the camera buffer to ensure
 * that we are returning the most recent frames
 *
 * @param camera the camera object
 */
void FlushCameraBuffer(dc1394camera_t *camera) {
    dc1394error_t err;
    dc1394video_frame_t *frame;

    int counter = 0;

    while (true) {
        err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_POLL, &frame);
        DC1394_WRN(err,"Could not capture a frame");

        if (frame != NULL) {
            err = dc1394_capture_enqueue(camera, frame);
            DC1394_WRN(err,"releasing buffer after failure");
            counter ++;
        } else {
            // the buffer is empty, we're done
            break;
        }
    }
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}


/**
 * Parses stereo configuration file to read parameters
 *
 * @param configFile path to the configuration file
 * @param configStruct pointer to an initialized OpenCvStereoConfig structure that will be filled in.
 *
 * @retval true on success, false on failure
 */

bool ParseConfigFile(string configFile, OpenCvStereoConfig *configStruct)
{
    // parse configuration file to get camera parameters
    GKeyFile *keyfile;
    GKeyFileFlags configFlags = G_KEY_FILE_NONE;
    GError *gerror = NULL;

    /* Create a new GKeyFile object and a bitwise list of flags. */
    keyfile = g_key_file_new ();

    /* Load the GKeyFile from "configFile" or return. */
    if (!g_key_file_load_from_file (keyfile, configFile.c_str(), configFlags, &gerror))
    {
        fprintf(stderr, "Configuration file \"%s\" not found.\n", configFile.c_str());
        return false;
    }

    // read parameters
    //uint64         guid = 0x00b09d0100a01a9a; // camera 1
    //uint64         guid2 = 0x00b09d0100a01ac5; // camera2

    // configuration file can't read 0x.... format so we get GUIDs as
    // a string and convert

    char *guidChar = g_key_file_get_string(keyfile, "cameras", "left", NULL);
    if (guidChar == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify guid for left camera (or I failed to read it). Parameter: cameras.left\n");
        return false;
    }
    // convert it to a uint64

    std::stringstream ss;
    ss << std::hex << guidChar;
    ss >> configStruct->guidLeft;


    char *guidChar2 = g_key_file_get_string(keyfile, "cameras", "right", NULL);
    if (guidChar == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify guid for right camera (or I failed to read it). Parameter: cameras.right\n");
        return false;
    }
    // convert it to a uint64

    std::stringstream ss2;
    ss2 << std::hex << guidChar2;
    ss2 >> configStruct->guidRight;

    char *stereoControlChannel = g_key_file_get_string(keyfile, "lcm", "stereo_control_channel", NULL);

    if (stereoControlChannel == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify stereo_control_channel (or I failed to read it). Parameter: lcm.stereo_control_channel\n");
        return false;
    }
    configStruct->stereoControlChannel = stereoControlChannel;


    const char *stereo_replay_channel = g_key_file_get_string(keyfile, "lcm", "stereo_replay_channel", NULL);

    if (stereo_replay_channel == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify stereo_replay_channel (or I failed to read it). Parameter: lcm.stereo_replay_channel\n");

        // this is not a fatal error, don't bail out
        stereo_replay_channel = "";
    }
    configStruct->stereo_replay_channel = stereo_replay_channel;


    const char *baro_airspeed_channel = g_key_file_get_string(keyfile, "lcm", "baro_airspeed_channel", NULL);

    if (baro_airspeed_channel == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify baro_airspeed_channel (or I failed to read it). Parameter: lcm.baro_airspeed_channel\n");

        // this is not a fatal error, don't bail out
        baro_airspeed_channel = "";
    }
    configStruct->baro_airspeed_channel = baro_airspeed_channel;

    const char *battery_status_channel = g_key_file_get_string(keyfile, "lcm", "battery_status_channel", NULL);

    if (battery_status_channel == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify battery_status_channel (or I failed to read it). Parameter: lcm.battery_status_channel\n");

        // this is not a fatal error, don't bail out
        battery_status_channel = "";
    }
    configStruct->battery_status_channel = battery_status_channel;

    const char *servo_out_channel = g_key_file_get_string(keyfile, "lcm", "servo_out_channel", NULL);

    if (servo_out_channel == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify servo_out_channel (or I failed to read it). Parameter: lcm.servo_out_channel\n");

        // this is not a fatal error, don't bail out
        servo_out_channel = "";
    }
    configStruct->servo_out_channel = servo_out_channel;

    const char *optotrak_channel = g_key_file_get_string(keyfile, "lcm", "optotrak_channel", NULL);

    if (optotrak_channel == NULL)
    {
        //fprintf(stderr, "Warning: configuration file does not specify optotrak_channel (or I failed to read it). Parameter: lcm.optotrak_channel\n");

        // this is not a fatal error, don't bail out
        optotrak_channel = "";
    }
    configStruct->optotrak_channel = optotrak_channel;

    const char *pose_channel = g_key_file_get_string(keyfile, "lcm", "pose_channel", NULL);

    if (pose_channel == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify pose_channel (or I failed to read it). Parameter: lcm.pose_channel\n");

        // this is not a fatal error, don't bail out
        pose_channel = "";
    }
    configStruct->pose_channel = pose_channel;

    const char *gps_channel = g_key_file_get_string(keyfile, "lcm", "gps_channel", NULL);

    if (gps_channel == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify gps_channel (or I failed to read it). Parameter: lcm.gps_channel\n");

        // this is not a fatal error, don't bail out
        gps_channel = "";
    }
    configStruct->gps_channel = gps_channel;

    const char *cpu_info_channel1 = g_key_file_get_string(keyfile, "lcm", "cpu_info_channel1", NULL);

    if (cpu_info_channel1 == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify cpu_info_channel1 (or I failed to read it). Parameter: lcm.cpu_info_channel1\n");

        // this is not a fatal error, don't bail out
        cpu_info_channel1 = "";
    }
    configStruct->cpu_info_channel1 = cpu_info_channel1;

    const char *cpu_info_channel2 = g_key_file_get_string(keyfile, "lcm", "cpu_info_channel2", NULL);

    if (cpu_info_channel2 == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify cpu_info_channel2 (or I failed to read it). Parameter: lcm.cpu_info_channel2\n");

        // this is not a fatal error, don't bail out
        cpu_info_channel2 = "";
    }
    configStruct->cpu_info_channel2 = cpu_info_channel2;

    const char *cpu_info_channel3 = g_key_file_get_string(keyfile, "lcm", "cpu_info_channel3", NULL);

    if (cpu_info_channel3 == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify cpu_info_channel3 (or I failed to read it). Parameter: lcm.cpu_info_channel3\n");

        // this is not a fatal error, don't bail out
        cpu_info_channel3 = "";
    }
    configStruct->cpu_info_channel3 = cpu_info_channel3;


    const char *log_size_channel1 = g_key_file_get_string(keyfile, "lcm", "log_size_channel1", NULL);

    if (log_size_channel1 == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify log_size_channel1 (or I failed to read it). Parameter: lcm.log_size_channel1\n");

        // this is not a fatal error, don't bail out
        log_size_channel1 = "";
    }
    configStruct->log_size_channel1 = log_size_channel1;

    const char *log_size_channel2 = g_key_file_get_string(keyfile, "lcm", "log_size_channel2", NULL);

    if (log_size_channel2 == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify log_size_channel2 (or I failed to read it). Parameter: lcm.log_size_channel2\n");

        // this is not a fatal error, don't bail out
        log_size_channel2 = "";
    }
    configStruct->log_size_channel2 = log_size_channel2;

    const char *log_size_channel3 = g_key_file_get_string(keyfile, "lcm", "log_size_channel3", NULL);

    if (log_size_channel3 == NULL)
    {
        fprintf(stderr, "Warning: configuration file does not specify log_size_channel3 (or I failed to read it). Parameter: lcm.log_size_channel3\n");

        // this is not a fatal error, don't bail out
        log_size_channel3 = "";
    }
    configStruct->log_size_channel3 = log_size_channel3;



    char *lcmUrl = g_key_file_get_string(keyfile, "lcm", "url", NULL);
    if (lcmUrl == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify URL for LCM (or I failed to read it). Parameter: lcm.url\n");
        return false;
    }

    configStruct->lcmUrl = lcmUrl;


    // get the calibration directory
    char *calibDir = g_key_file_get_string(keyfile, "cameras", "calibrationDir", NULL);
    if (calibDir == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify calibration directory (or I failed to read it). Parameter: cameras.calibDir\n");
        return false;
    }

    configStruct->calibrationDir = ReplaceUserVarInPath(calibDir);

    configStruct->lastValidPixelRow = g_key_file_get_integer(keyfile,
        "cameras", "lastValidPixelRow", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify the last valid row of pixels (or I failed to read it). Parameter: cameras.lastValidPixelRow\n");

        g_error_free(gerror);

        return false;
    }

    // get the video saving directory
    const char *videoSaveDir = g_key_file_get_string(keyfile, "cameras", "videoSaveDir", NULL);
    if (videoSaveDir == NULL)
    {
        // default to the current directory
        videoSaveDir = ".";
    }
    configStruct->videoSaveDir = videoSaveDir;

    // get the fourcc video codec
    char *fourcc = g_key_file_get_string(keyfile, "cameras", "fourcc", NULL);
    if (fourcc == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify fourcc (or I failed to read it). Parameter: cameras.fourcc\n");
        return false;
    }
    configStruct->fourcc = fourcc;

    // get the fourcc video codec
    bool usePGM = g_key_file_get_boolean(keyfile, "cameras", "usePGM", &gerror);
    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify usePGM (or I failed to read it). Parameter: cameras.usePGM\n");
        g_error_free(gerror);
        return false;
    }
    configStruct->usePGM = usePGM;



    configStruct->displayOffsetX = g_key_file_get_integer(keyfile,
        "display", "offset_x", &gerror);
    if (gerror != NULL)
    {
        // no need to get upset, this is an optional parameter
        configStruct->displayOffsetX = 0;
        g_error_free(gerror);
        gerror = NULL;
    }

    configStruct->displayOffsetY = g_key_file_get_integer(keyfile,
        "display", "offset_y", &gerror);
    if (gerror != NULL)
    {
        // no need to get upset, this is an optional parameter
        configStruct->displayOffsetY = 0;
        g_error_free(gerror);
        gerror = NULL;
    }


    // get settings

    // get disparity
    configStruct->disparity = g_key_file_get_integer(keyfile,
        "settings", "disparity", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify disparity (or I failed to read it). Parameter: settings.disparity\n");

        g_error_free(gerror);

        return false;
    }

    configStruct->infiniteDisparity = g_key_file_get_integer(keyfile,
        "settings", "infiniteDisparity", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify infinite disparity (or I failed to read it). Parameter: settings.infiniteDisparity\n");

        g_error_free(gerror);

        return false;
    }


    // get interestOperatorLimit
    configStruct->interestOperatorLimit =
        g_key_file_get_integer(keyfile, "settings",
        "interestOperatorLimit", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify interest operator limit (or I failed to read it). Parameter: settings.interestOperatorLimit\n");

        g_error_free(gerror);

        return false;
    }

    configStruct->horizontalInvarianceMultiplier =
        g_key_file_get_double(keyfile, "settings",
        "horizontalInvarianceMultiplier", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify unit conversion (or I failed to read it). Parameter: settings.horizontalInvarianceMultiplier\n");

        g_error_free(gerror);

        return false;
    }

    // get blockSize
    configStruct->blockSize =
        g_key_file_get_integer(keyfile, "settings",
        "blockSize", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify block size (or I failed to read it). Parameter: settings.blockSize\n");

        g_error_free(gerror);

        return false;
    }

    // get sadThreshold
    configStruct->sadThreshold =
        g_key_file_get_integer(keyfile, "settings",
        "sadThreshold", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify SAD threshold (or I failed to read it). Parameter: settings.sadThreshold\n");

        g_error_free(gerror);

        return false;
    }

    configStruct->calibrationUnitConversion =
        g_key_file_get_double(keyfile, "cameras",
        "calibrationUnitConversion", &gerror);

    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify unit conversion (or I failed to read it). Parameter: cameras.calibrationUnitConversion\n");

        g_error_free(gerror);

        return false;
    }

    return true;
}

/**
 * Loads XML stereo calibration files and reamps them.
 *
 * @param calibrationDir directory the calibration files are in
 * @param stereoCalibration calibration structure to fill in
 *
 * @retval true on success, false on falure.
 *
 */
bool LoadCalibration(string calibrationDir, OpenCvStereoCalibration *stereoCalibration)
{
    Mat qMat, mx1Mat, my1Mat, mx2Mat, my2Mat, m1Mat, d1Mat, r1Mat, p1Mat, r2Mat, p2Mat, m2Mat, d2Mat;

    CvMat *Q = (CvMat *)cvLoad((calibrationDir + "/Q.xml").c_str(),NULL,NULL,NULL);

    if (Q == NULL)
    {
        std::cerr << "Error: failed to read " + calibrationDir + "/Q.xml." << std::endl;
        return false;
    }

    CvMat *mx1 = (CvMat *)cvLoad((calibrationDir + "/mx1.xml").c_str(),NULL,NULL,NULL);

    if (mx1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/mx1.xml." << std::endl;
        return false;
    }

    CvMat *my1 = (CvMat *)cvLoad((calibrationDir + "/my1.xml").c_str(),NULL,NULL,NULL);

    if (my1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/my1.xml." << std::endl;
        return false;
    }

    CvMat *mx2 = (CvMat *)cvLoad((calibrationDir + "/mx2.xml").c_str(),NULL,NULL,NULL);

    if (mx2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/mx2.xml." << std::endl;
        return false;
    }

    CvMat *my2 = (CvMat *)cvLoad((calibrationDir + "/my2.xml").c_str(),NULL,NULL,NULL);

    if (my2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/my2.xml." << std::endl;
        return false;
    }

    CvMat *m1 = (CvMat *)cvLoad((calibrationDir + "/M1.xml").c_str(),NULL,NULL,NULL);

    if (m1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/M1.xml." << std::endl;
        return false;
    }

    CvMat *d1 = (CvMat *)cvLoad((calibrationDir + "/D1.xml").c_str(),NULL,NULL,NULL);

    if (d1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/D1.xml." << std::endl;
        return false;
    }

    CvMat *r1 = (CvMat *)cvLoad((calibrationDir + "/R1.xml").c_str(),NULL,NULL,NULL);

    if (r1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/R1.xml." << std::endl;
        return false;
    }

    CvMat *p1 = (CvMat *)cvLoad((calibrationDir + "/P1.xml").c_str(),NULL,NULL,NULL);

    if (p1 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/P1.xml." << std::endl;
        return false;
    }

    CvMat *m2 = (CvMat *)cvLoad((calibrationDir + "/M2.xml").c_str(),NULL,NULL,NULL);

    if (m2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/M2.xml." << std::endl;
        return false;
    }

    CvMat *d2 = (CvMat *)cvLoad((calibrationDir + "/D2.xml").c_str(),NULL,NULL,NULL);

    if (d2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/D2.xml." << std::endl;
        return false;
    }

    CvMat *r2 = (CvMat *)cvLoad((calibrationDir + "/R2.xml").c_str(),NULL,NULL,NULL);

    if (r2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/R2.xml." << std::endl;
        return false;
    }

    CvMat *p2 = (CvMat *)cvLoad((calibrationDir + "/P2.xml").c_str(),NULL,NULL,NULL);

    if (p2 == NULL)
    {
        std::cerr << "Error: failed to read " << calibrationDir << "/P2.xml." << std::endl;
        return false;
    }



    qMat = Mat(Q, true);
    mx1Mat = Mat(mx1,true);
    my1Mat = Mat(my1,true);
    mx2Mat = Mat(mx2,true);
    my2Mat = Mat(my2,true);

    m1Mat = Mat(m1,true);
    d1Mat = Mat(d1,true);
    r1Mat = Mat(r1,true);
    p1Mat = Mat(p1,true);

    m2Mat = Mat(m2,true);
    d2Mat = Mat(d2,true);
    r2Mat = Mat(r2,true);
    p2Mat = Mat(p2,true);

    Mat mx1fp, empty1, mx2fp, empty2;

    // this will convert to a fixed-point notation
    convertMaps(mx1Mat, my1Mat, mx1fp, empty1, CV_16SC2, true);
    convertMaps(mx2Mat, my2Mat, mx2fp, empty2, CV_16SC2, true);

    stereoCalibration->qMat = qMat;
    stereoCalibration->mx1fp = mx1fp;
    stereoCalibration->mx2fp = mx2fp;

    stereoCalibration->M1 = m1Mat;
    stereoCalibration->D1 = d1Mat;
    stereoCalibration->R1 = r1Mat;
    stereoCalibration->P1 = p1Mat;

    stereoCalibration->M2 = m2Mat;
    stereoCalibration->D2 = d2Mat;
    stereoCalibration->R2 = r2Mat;
    stereoCalibration->P2 = p2Mat;


    return true;
}

/**
 * Stops capture on a camera and frees its context.
 *
 * @param d dc1394_t context
 * @param camera dc1394camera_t for the camera
 *
 */
void StopCapture(dc1394_t *dcContext, dc1394camera_t *camera)
{

    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);

    dc1394_free (dcContext);
}









void InitBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool enable_gamma) {

    // set absolute control to off
    if (camera1 != NULL) {
        dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_GAIN, DC1394_OFF);
        dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_SHUTTER, DC1394_OFF);
        dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
        dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_BRIGHTNESS, DC1394_OFF);

        // set auto/manual settings
        dc1394_feature_set_mode(camera1, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera1, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera1, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);

        dc1394_feature_set_mode(camera1, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);

        dc1394_feature_set_mode(camera1, DC1394_FEATURE_FRAME_RATE, DC1394_FEATURE_MODE_AUTO);

        if (enable_gamma) {
            dc1394_feature_set_power(camera1, DC1394_FEATURE_GAMMA, DC1394_ON);
        } else {
            dc1394_feature_set_power(camera1, DC1394_FEATURE_GAMMA, DC1394_OFF);
        }
    }

    if (camera2 != NULL) {
        dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_GAIN, DC1394_OFF);

        dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_SHUTTER, DC1394_OFF);

        dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_EXPOSURE, DC1394_OFF);

        dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_BRIGHTNESS, DC1394_OFF);
    }


    // for camera 2 (slave on brightness settings), set everything
    // to manual except framerate

    if (camera2) {
        dc1394_feature_set_mode(camera2, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera2, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera2, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera2, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera2, DC1394_FEATURE_FRAME_RATE, DC1394_FEATURE_MODE_AUTO);

        if (enable_gamma) {
            dc1394_feature_set_power(camera2, DC1394_FEATURE_GAMMA, DC1394_ON);
        } else {
            dc1394_feature_set_power(camera2, DC1394_FEATURE_GAMMA, DC1394_OFF);
        }
    }

}

/**
 * Send an image over LCM (useful for viewing in a
 * headless configuration
 *
 * @param lcm already initialized lcm object
 * @param channel channel name to send over
 * @param image the image to send
 * @param compression_quality 0-100 for jpeg compression quality. Set to -1 for no compression.
 *      Default: 80
 *
 */
void SendImageOverLcm(lcm_t* lcm, string channel, Mat image, int compression_quality) {

    if (!image.isContinuous()) {
        //std::cout << "Image not continuous. LCM transport not implemented." << std::endl;
        //return;

        image = image.clone();

    }

    // create LCM message
    bot_core_image_t msg;

    msg.utime = getTimestampNow();

    msg.width = image.cols;
    msg.height = image.rows;
    msg.row_stride = image.cols;

    msg.nmetadata = 0;

    if (compression_quality >= 0) {

        if (image.type() == CV_8UC1) {
            // use jpeg compression

            // allocate a buffer to put the result into
            int bufsize = image.cols * image.rows;
            uint8_t buffer[bufsize];

            jpeg_compress_8u_gray(image.ptr(), image.cols, image.rows, image.step, buffer, &bufsize, compression_quality);

            // got the compressed file, now push it to LCM
            msg.data = buffer;
            msg.size = bufsize;

            msg.pixelformat = 1196444237; // see bot_core_image_t.lcm --> PIXEL_FORMAT_MJPEG

            // send the image over lcm
            bot_core_image_t_publish(lcm, channel.c_str(), &msg);

        } else if (image.type() == CV_8UC3) {

            // allocate a buffer to put the result into
            int bufsize = image.cols * image.rows * 3;
            uint8_t buffer[bufsize];

            Mat rgb_mat;

            cvtColor(image, rgb_mat, CV_BGR2RGB);

            jpeg_compress_8u_rgb(image.ptr(), image.cols, image.rows, image.step, buffer, &bufsize, compression_quality);

            msg.data = buffer;
            msg.size = bufsize;

            msg.pixelformat = 1196444237; // see bot_core_image_t.lcm --> PIXEL_FORMAT_MJPEG

            // send the image over lcm
            bot_core_image_t_publish(lcm, channel.c_str(), &msg);




        } else {
            std::cout << "Image type not supported. LCM transport not implemented." << std::endl;
            return;
        }

    } else {

        if (image.type() == CV_8UC1) {
            msg.pixelformat = 1497715271; // see bot_core_image_t.lcm --> PIXEL_FORMAT_GRAY; // TODO: detect this

            msg.data = image.ptr();

            msg.size = image.cols * image.rows;

            // send the image over lcm
            bot_core_image_t_publish(lcm, channel.c_str(), &msg);
        } else if (image.type() == CV_8UC3) {

            Mat rgb_mat;

            cvtColor(image, rgb_mat, CV_BGR2RGB);


            msg.pixelformat = 859981650; // PIXEL_FORMAT_RGB

            msg.data = rgb_mat.ptr();

            msg.size = rgb_mat.cols * rgb_mat.rows * 3;

            msg.row_stride = image.step;

            bot_core_image_t_publish(lcm, channel.c_str(), &msg);

        } else {
            std::cout << "Image type not supported. LCM transport not implemented." << std::endl;
            return;
        }

    }

}

/**
 * Takes an lcm_stereo message and produces a vector of Point3fs corresponding to the points contained
 * in the message
 *
 * @param msg lcm_stereo message
 * @param points_out vector<Point3f> where the points will be placed.
 */
void Get3DPointsFromStereoMsg(const lcmt_stereo *msg, vector<Point3f> *points_out)
{
    for (int i=0; i<msg->number_of_points; i++)
    {
        points_out->push_back(Point3f(msg->x[i], msg->y[i], msg->z[i]));
    }
}

/**
 * Draws 3D points onto a 2D image when given the camera calibration.
 *
 * @param camera_image image to draw onto
 * @param points_list_in vector<Point3f> of 3D points to draw. Likely obtained from Get3DPointsFromStereoMsg
 * @param cam_mat_m camera calibration matrix (usually M1.xml)
 * @param cam_mat_d distortion calibration matrix (usually D1.xml)
 * @param cam_mat_r rotation calibration matrix (usually R1.xml)
 * @param outline_color color to draw the box outlines (default: 128)
 * @param inside_color color to draw the inside of the boxes (default: 255). Set to -1 for no fill.
 * @param box_top if you only want to draw points inside a box, this specifies one coordinate of the box
 * @param box_bottom the second coordinate of the box
 * @param points_in_box if you pass box_top and box_bottom, this will be filled with the indicies of
 *          the points inside the box.
 * @param min_z minimum z value allowable to draw the point
 * @param max_z maximum z value allowable to draw the point
 * @param box_size size of the box (default = 4)
 */
void Draw3DPointsOnImage(Mat camera_image, vector<Point3f> *points_list_in, Mat cam_mat_m, Mat cam_mat_d, Mat cam_mat_r, Scalar outline_color, Scalar inside_color, Point2d box_top, Point2d box_bottom, vector<int> *points_in_box,
float min_z, float max_z, int box_size) {
    vector<Point3f> &points_list = *points_list_in;

    if (points_list.size() <= 0)
    {
        //std::cout << "Draw3DPointsOnimage: zero sized points list" << std::endl;
        return;
    }


    vector<Point2f> img_points_list;

    projectPoints(points_list, cam_mat_r.inv(), Mat::zeros(3, 1, CV_32F), cam_mat_m, cam_mat_d, img_points_list);


    int min_x = min(box_top.x, box_bottom.x);
    int min_y = min(box_top.y, box_bottom.y);
    int max_x = max(box_top.x, box_bottom.x);
    int max_y = max(box_top.y, box_bottom.y);
    bool box_bounding = false;

    if (box_top.x != -1 || box_top.y != -1 || box_bottom.x != -1 || box_bottom.y != -1) {
        box_bounding = true;
    }

    int thickness = CV_FILLED;
    if (inside_color[0] == -1) {
        thickness = 1;
    }

    // now draw the points onto the image
    for (int i=0; i<int(img_points_list.size()); i++)
    {

        //line(camera_image, Point(img_points_list[i].x, 0), Point(img_points_list[i].x, camera_image.rows), color);
        //line(camera_image, Point(0, img_points_list[i].y), Point(camera_image.cols, img_points_list[i].y), color);

        bool flag = false;

        if (box_bounding) {

            if (img_points_list[i].x >= min_x && img_points_list[i].x <= max_x &&
                img_points_list[i].y >= min_y && img_points_list[i].y <= max_y) {

                if (points_in_box) {
                    points_in_box->push_back(i);
                }

                flag = true;
            }
        }


        if (box_bounding == false || flag == true) {

            if (min_z == 0 || points_list[i].z >= min_z) {

                if (max_z == 0 || points_list[i].z <= max_z) {

                    rectangle(camera_image, Point(img_points_list[i].x - box_size, img_points_list[i].y - box_size),
                        Point(img_points_list[i].x + box_size, img_points_list[i].y + box_size), outline_color, thickness);

                    if (inside_color[0] != -1) {
                        rectangle(camera_image, Point(img_points_list[i].x - 2, img_points_list[i].y - box_size/2),
                            Point(img_points_list[i].x + box_size/2, img_points_list[i].y + box_size/2), inside_color, thickness);
                    }

                }
            }
        }

    }
}

int GetDisparityForDistance(double distance, const OpenCvStereoCalibration &calibration, int *inf_disparity) {

    int min_search = -100;
    int max_search = 100;

    cv::vector<Point3f> disparity_candidates;
    cv::vector<int> disparity_values;
    for (int i = min_search; i <= max_search; i++) {
        disparity_candidates.push_back(Point3f(0, 0, -i)); // note the negative! it is correct!
        disparity_values.push_back(i);
    }
    cv::vector<Point3f> vector_3d_out;

    perspectiveTransform(disparity_candidates, vector_3d_out, calibration.qMat);

    int best_disparity = 0;
    double best_dist_abs = -1;
    double max_dist = -1000;
    int max_disparity = 0;

    for (int i = 0; i < (int)vector_3d_out.size(); i++) {
        double this_dist_abs = fabs(vector_3d_out.at(i).z - distance);
        //std::cout << "Disp: " << disparity_values.at(i) << " ----> " << vector_3d_out.at(i).z << " (dist = " << this_dist_abs << ")" << std::endl;
        if (best_dist_abs == -1 || this_dist_abs < best_dist_abs) {
            best_disparity = disparity_values.at(i);
            best_dist_abs = this_dist_abs;
        }

        if (vector_3d_out.at(i).z > max_dist) {
            max_dist = vector_3d_out.at(i).z;
            max_disparity = disparity_values.at(i);
        }
    }

    if (inf_disparity != nullptr) {
        *inf_disparity = max_disparity;
    }

    return best_disparity;
}


/**
 * We need to make sure the brightness settings
 * are identical on both cameras. to do this, we
 * read the settings off the left camera, and force
 * the right camera to do the exact same thing.
 * since we want auto-brightness, we do this every frame
 *
 * in practice, it is really important to get this right
 * otherwise the pixel values will be totally different
 * and stereo will not work at all
 *
 * @param camera1 first camera that we will read settings from it's automatic features
 * @param camera2 camera that will we transfer settings to
 *
 * @param completeSet if true, will take longer but set more parameters. Useful on initialization or if brightness is expected to change a lot (indoors to outdoors)
 *
 */
void MatchBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool complete_set, int force_brightness, int force_exposure)
{


    #if 0
    dc1394error_t err;

    //dc1394_feature_print(&features.feature[8], stdout);


    // set brightness
    dc1394_feature_set_value(camera2, features.feature[0].id, features.feature[0].value);

    // set exposure
    dc1394_feature_set_value(camera2, features.feature[1].id, features.feature[1].value);

    // set gamma
    dc1394_feature_set_value(camera2, features.feature[6].id, features.feature[6].value);

    // set shutter
    dc1394_feature_set_absolute_value(camera2, features.feature[7].id, features.feature[7].abs_value);
    #endif

    // set shutter
    uint32_t shutterVal;
    dc1394_feature_get_value(camera1, DC1394_FEATURE_SHUTTER, &shutterVal);
    dc1394_feature_set_value(camera2, DC1394_FEATURE_SHUTTER, shutterVal);



    // set gain
    uint32_t gainVal;
    dc1394_feature_get_value(camera1, DC1394_FEATURE_GAIN, &gainVal);

    dc1394_feature_set_value(camera2, DC1394_FEATURE_GAIN, gainVal);
    //DC1394_WRN(err,"Could not set gain");


    if (complete_set == true && force_brightness == -1 && force_exposure == -1)
    {
        // if this is true, we're willing to wait a while and
        // really get this right

        uint32_t exposure_value, brightness_value;

        // enable auto exposure on camera1
        dc1394_feature_set_mode(camera1, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_AUTO);

        dc1394_feature_set_mode(camera1, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_AUTO);

        // take a bunch of frames with camera1 to let it set exposure
        for (int i=0;i<25;i++)
        {
            Mat img = GetFrameFormat7(camera1);
        }

        // turn off auto exposure
        dc1394_feature_set_mode(camera1, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);

        dc1394_feature_set_mode(camera1, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);

        // get exposure value

        dc1394_feature_get_value(camera1, DC1394_FEATURE_EXPOSURE, &exposure_value);

        dc1394_feature_get_value(camera1, DC1394_FEATURE_BRIGHTNESS, &brightness_value);

        //std::cout << "exposure: " << exposure_value << " brightness: " << brightness_value << std::endl;

        // set exposure
        dc1394_feature_set_value(camera2, DC1394_FEATURE_EXPOSURE, exposure_value);

        dc1394_feature_set_value(camera2, DC1394_FEATURE_BRIGHTNESS, brightness_value);

    } else if (complete_set == true)
    {
        // at least one of the exposure / brightness settings is
        // not -1
        if (force_brightness < 0 || force_brightness > 254
            || force_exposure < 0 || force_exposure > 254)
        {
            printf("Error: brightness (%d) or exposure (%d) out of range, not settting.\n", force_brightness, force_exposure);
        } else {
            // set the brightness and exposure
            dc1394_feature_set_value(camera1, DC1394_FEATURE_EXPOSURE, force_exposure);

            dc1394_feature_set_value(camera1, DC1394_FEATURE_BRIGHTNESS, force_brightness);

            dc1394_feature_set_value(camera2, DC1394_FEATURE_EXPOSURE, force_exposure);

            dc1394_feature_set_value(camera2, DC1394_FEATURE_BRIGHTNESS, force_brightness);
        }
    }


    #if 0
    dc1394featureset_t features, features2;
    dc1394_feature_get_all(camera1, &features);

    // set framerate
    dc1394_feature_set_absolute_value(camera2, features.feature[15].id, features.feature[15].abs_value);

    dc1394_feature_get_all(camera2, &features2);



    std::cout << std::endl << dc1394_feature_get_string(features.feature[0].id) << ": " << features.feature[0].value << "/" << features2.feature[0].value << std::endl;


    // set exposure
    std::cout << std::endl << dc1394_feature_get_string(features.feature[1].id) << ": " << features.feature[1].value << "/" << features2.feature[1].value << std::endl;


    // set gamma
    std::cout << std::endl << dc1394_feature_get_string(features.feature[6].id) << ": " << features.feature[6].value << "/" << features2.feature[6].value << std::endl;


    // set shutter
    std::cout << std::endl << dc1394_feature_get_string(features.feature[7].id) << ": " << features.feature[7].value << "/" << features2.feature[7].value << std::endl;

    // set gain
   std::cout << std::endl << dc1394_feature_get_string(features.feature[8].id) << ": " << features.feature[8].value << "/" << features2.feature[8].value << std::endl;

    //std::cout << "---------------------------------------" << std::endl;
    //dc1394_feature_print_all(&features, stdout);
    //std::cout << "222222222222222222222222222222222222222222222" << std::endl;
    //dc1394_feature_print_all(&features2, stdout);
    //dc1394_feature_get_absolute_value(camera1, DC1394_FEATURE_BRIGHTNESS, &brightnessVal);

    //std::cout << std::endl << "brightness: " << brightnessVal << std::endl;
    #endif
}

