/*
 * Utility functions for opencv-stereo
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */
 
#include "opencv-stereo-util.hpp"


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
    
    stringstream ss;
    ss << std::hex << guidChar;
    ss >> configStruct->guidLeft;
    
    
    char *guidChar2 = g_key_file_get_string(keyfile, "cameras", "right", NULL);
    if (guidChar == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify guid for right camera (or I failed to read it). Parameter: cameras.right\n");
        return false;
    }
    // convert it to a uint64

    stringstream ss2;
    ss2 << std::hex << guidChar2;
    ss2 >> configStruct->guidRight;
    
    char *stereoControlChannel = g_key_file_get_string(keyfile, "lcm", "stereo_control_channel", NULL);
    
    if (stereoControlChannel == NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify stereo_control_channel (or I failed to read it). Parameter: lcm.stereo_control_channel\n");
        return false;
    }
    configStruct->stereoControlChannel = stereoControlChannel;
    
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
    
    configStruct->calibrationDir = calibDir;
    
    // get the video saving directory
    char *videoSaveDir = g_key_file_get_string(keyfile, "cameras", "videoSaveDir", NULL);
    if (videoSaveDir == NULL)
    {
        // default to the current directory
        strcpy(videoSaveDir, ".");
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
    
    return true;
}

/**
 * Loads XML stereo calibration files and reamps them.
 *
 * @param calibrationDir directory the calibration files are in
 * @param mx1fp CV_16SC2 mx1 remap to be assigned
 * @param mx2fp CV_16SC2 mx2 remap to be assigned
 * @param qMat matrix from Q.xml to be assigned
 *
 * @retval true on success, false on falure.
 *
 */
bool LoadCalibration(string calibrationDir, OpenCvStereoCalibration *stereoCalibration)
{
    Mat qMat, mx1Mat, my1Mat, mx2Mat, my2Mat;

    CvMat *Q = (CvMat *)cvLoad((calibrationDir + "/Q.xml").c_str(),NULL,NULL,NULL);
    
    if (Q == NULL)
    {
        cerr << "Error: failed to read " + calibrationDir + "/Q.xml." << endl;
        return false;
    }
    
    CvMat *mx1 = (CvMat *)cvLoad((calibrationDir + "/mx1.xml").c_str(),NULL,NULL,NULL);
    
    if (mx1 == NULL)
    {
        cerr << "Error: failed to read " << calibrationDir << "/mx1.xml." << endl;
        return false;
    }
    
    CvMat *my1 = (CvMat *)cvLoad((calibrationDir + "/my1.xml").c_str(),NULL,NULL,NULL);
    
    if (my1 == NULL)
    {
        cerr << "Error: failed to read " << calibrationDir << "/my1.xml." << endl;
        return false;
    }
    
    CvMat *mx2 = (CvMat *)cvLoad((calibrationDir + "/mx2.xml").c_str(),NULL,NULL,NULL);
    
    if (mx2 == NULL)
    {
        cerr << "Error: failed to read " << calibrationDir << "/mx2.xml." << endl;
        return false;
    }
    
    CvMat *my2 = (CvMat *)cvLoad((calibrationDir + "/my2.xml").c_str(),NULL,NULL,NULL);
    
    if (my2 == NULL)
    {
        cerr << "Error: failed to read " << calibrationDir << "/my2.xml." << endl;
        return false;
    }
    
    
    
    qMat = Mat(Q, true);
    mx1Mat = Mat(mx1,true);
    my1Mat = Mat(my1,true);
    mx2Mat = Mat(mx2,true);
    my2Mat = Mat(my2,true);
    
    Mat mx1fp, empty1, mx2fp, empty2;
    
    // this will convert to a fixed-point notation
    convertMaps(mx1Mat, my1Mat, mx1fp, empty1, CV_16SC2, true);
    convertMaps(mx2Mat, my2Mat, mx2fp, empty2, CV_16SC2, true);
    
    stereoCalibration->qMat = qMat;
    stereoCalibration->mx1fp = mx1fp;
    stereoCalibration->mx2fp = mx2fp;
    
    
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

/**
 * Sets up a video writer with the given filename
 *
 * @param filenamePrefix prefix for the name of the video file. It will be created with a date and time appended to the name.
 * @param frameSize size of the frames
 * @param configStruct OpenCvStereoConfig structure for reading the directory things should be saved in
 *
 * @retval VideoWriter object
 */
VideoWriter SetupVideoWriter(string filenamePrefix, Size frameSize, OpenCvStereoConfig configStruct)
{
    // get the date and time for the filename
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    strftime (buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);
    
    char fourcc1 = configStruct.fourcc.at(0);
    char fourcc2 = configStruct.fourcc.at(1);
    char fourcc3 = configStruct.fourcc.at(2);
    char fourcc4 = configStruct.fourcc.at(3);
    
    VideoWriter recorder(configStruct.videoSaveDir + "/" + filenamePrefix + "-" + string(buffer) + ".avi", CV_FOURCC(fourcc1, fourcc2, fourcc3, fourcc4), 30, frameSize, false);
    
    if( !recorder.isOpened() ) {
        printf("VideoWriter failed to open!\n");
    }
    return recorder;
}

void InitBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2)
{
    // set absolute control to off
    dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_GAIN, DC1394_OFF);
    dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_GAIN, DC1394_OFF);
    
    dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_SHUTTER, DC1394_OFF);
    dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_SHUTTER, DC1394_OFF);
    
    dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
    dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_EXPOSURE, DC1394_OFF);
    
    dc1394_feature_set_absolute_control(camera1, DC1394_FEATURE_BRIGHTNESS, DC1394_OFF);
    dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_BRIGHTNESS, DC1394_OFF);
    
    // set auto/manual settings
    dc1394_feature_set_mode(camera1, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    
    dc1394_feature_set_mode(camera1, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
    
    dc1394_feature_set_mode(camera1, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_AUTO);
    
    dc1394_feature_set_mode(camera1, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_AUTO);
    
    dc1394_feature_set_mode(camera1, DC1394_FEATURE_FRAME_RATE, DC1394_FEATURE_MODE_AUTO);
    
    // for camera 2 (slave on brightness settings), set everything
    // to manual except framerate
    dc1394_feature_set_mode(camera2, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    
    dc1394_feature_set_mode(camera2, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
    
    dc1394_feature_set_mode(camera2, DC1394_FEATURE_SHUTTER, DC1394_FEATURE_MODE_MANUAL);
    
    dc1394_feature_set_mode(camera2, DC1394_FEATURE_GAIN, DC1394_FEATURE_MODE_MANUAL);
    
    dc1394_feature_set_mode(camera2, DC1394_FEATURE_FRAME_RATE, DC1394_FEATURE_MODE_AUTO);
}

