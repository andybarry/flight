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
 * @param camera the camrea
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
        cout << "Warning: failed to capture a frame, returning black frame." << endl;
        
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
    
    // get interestOperatorDivisor
    configStruct->interestOperatorDivisor =
        g_key_file_get_double(keyfile, "settings",
        "interestOperatorDivisor", &gerror);
    
    if (gerror != NULL)
    {
        fprintf(stderr, "Error: configuration file does not specify interest operator divisor (or I failed to read it). Parameter: settings.interestOperatorDivisor\n");
        
        g_error_free(gerror);
        
        return false;
    }
    
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
    Mat qMat, mx1Mat, my1Mat, mx2Mat, my2Mat, m1Mat, d1Mat;

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
    
    CvMat *m1 = (CvMat *)cvLoad((calibrationDir + "/M1.xml").c_str(),NULL,NULL,NULL);
    
    if (m1 == NULL)
    {
        cerr << "Error: failed to read " << calibrationDir << "/M1.xml." << endl;
        return false;
    }
    
    CvMat *d1 = (CvMat *)cvLoad((calibrationDir + "/D1.xml").c_str(),NULL,NULL,NULL);
    
    if (m1 == NULL)
    {
        cerr << "Error: failed to read " << calibrationDir << "/D1.xml." << endl;
        return false;
    }
    
    
    
    qMat = Mat(Q, true);
    mx1Mat = Mat(mx1,true);
    my1Mat = Mat(my1,true);
    mx2Mat = Mat(mx2,true);
    my2Mat = Mat(my2,true);
    
    m1Mat = Mat(m1,true);
    d1Mat = Mat(d1,true);
    
    Mat mx1fp, empty1, mx2fp, empty2;
    
    // this will convert to a fixed-point notation
    convertMaps(mx1Mat, my1Mat, mx1fp, empty1, CV_16SC2, true);
    convertMaps(mx2Mat, my2Mat, mx2fp, empty2, CV_16SC2, true);
    
    stereoCalibration->qMat = qMat;
    stereoCalibration->mx1fp = mx1fp;
    stereoCalibration->mx2fp = mx2fp;
    
    stereoCalibration->M1 = m1Mat;
    stereoCalibration->D1 = d1Mat;
    
    
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
 * Gets next availible filename for a video file
 *
 * @param configStruct OpenCvStereoConfig structure for reading the directory things should be saved in
 *  Default: false
 * @param increment_number true if we should increment video number
 *  Default: true
 *
 * @retval number of the next availible name
 *
 */
int GetNextVideoNumber(OpenCvStereoConfig configStruct, bool increment_number) {
    
    string datechar = GetDateSring();
    
    int max_number = 0;
    
    // other videos have been taken today
    if (boost::filesystem::exists(configStruct.videoSaveDir)) {
            
        max_number = MatchVideoFile(configStruct.videoSaveDir,  datechar, !configStruct.usePGM);
        
    }
    
    if (increment_number == true) {
        return max_number + 1;
    } else {
        return max_number;
    }
}

/**
 * Searches a directory for video files of the form 
 * video[L|R]-skip-xxx-xxxx-xx-xx.xx.avi
 * so for example, videoL-skip-0-2013-11-16.00.avi
 * 
 * If you give a match_number, we will return the skip value or -1 on failure
 * 
 * If you do not supply a match_number, we will return the maximum video file
 * number in the directory.
 * 
 * @param directory directory to search for videos
 * @param datestr string containing date to search for (ex. 2013-11-16)
 * @param using_avi if true, match for .avi, otherwise without that (ie for a directory)
 * @param match_number (optional).  If provided, will return the skip value for
 *  that number.  Otherwise will return the largest video number in the
 *  directory.
 * 
 * @retval if match_number, the skip value, else, the largest video number in
 *  the directory.
 * 
 */
int MatchVideoFile(string directory, string datestr, bool using_avi, int match_number) {
    
    int return_number = 0;
    
    boost::filesystem::directory_iterator end_itr; // default construction
                                                   // yields past-the-end
    for (boost::filesystem::directory_iterator itr(directory);
        itr != end_itr; ++itr ) {
        
        // iterate through the videos, keeping track
        // of the highest number
        
        // munch on the filename to pull out
        // just the ending part if using_avi
        
        // 17 characters in 2013-11-21.01.avi
        // or in            xxxx-xx-xx.xx.avi
        
        int avi_offset; 
        if (using_avi) {
            avi_offset = 0;
        } else {
            avi_offset = -4; // everything is 4 characters less since we don't have ".avi" at the end
        }
        
        string this_file = itr->path().leaf().string();
        
        if (int(this_file.length()) > 18 + avi_offset) {
        
            // might be a video file

            // read the date in the string
            string file_date = this_file.substr(
                this_file.length() - (17 + avi_offset), 10);
                
            // compare the date string and see if it
            // is today
            if (file_date.compare(datestr) == 0) {
            
                // matches date
                // get the number
                string file_number = this_file.substr(
                    this_file.length() - (6 + avi_offset), 2);
                    
                // attempt to convert the string to a number
                try {
                    int this_number = stoi(file_number);
                    
                    if (match_number >= 0) {
                        // if we are checking for a specific file, 
                        // find the skip amount in that file
                        
                        if (match_number == this_number) {
                            // this is the file we're searching for!
                            return GetSkipNumber(this_file);
                        }
                        
                    } else {
                        // if we are not checking for a match, determine if this
                        // is the max number
                        if (return_number < this_number) {
                            return_number = this_number;
                        }
                    }
                } catch (...) {
                    // failed to convert, don't do anything
                    // since this isn't
                }
            }
        }
    }
    
    return return_number;
}

/**
 * Reads a video filename and returns the skip number
 * 
 * @param filename video file name in the standard video filename format
 *  (ex. videoL-skip-0-2013-11-16.00.avi)
 * 
 * @retval skip amount or -1 on failure.
 * 
 */
int GetSkipNumber(string filename) {
    
    // videoL-skip- is 12 characters
    string filename2 = filename.substr(12);
    
    
    // now we have 226-2013-11-16.00.avi
    
    // need to match the first "-" and then we'll have our number
    int dash_pos = filename2.find("-");
    
    if ((unsigned int)dash_pos == string::npos) {
        // failed to find it, bail out
        return -1;
    }
    
    // get just the numbers
    string skipstr = filename2.substr(0, dash_pos);
    
    // convert to an integer
    try {
        int this_number = stoi(skipstr);
        
        return this_number;
    } catch (...) {
        // failed to convert, don't do anything
        // since this isn't
        return -1;
    }
    
    return -1;
}


/**
 * Returns the current date for use in video filenames
 *
 * @retval string of the date in "yyyy-mm-dd" format
 */
string GetDateSring() {
    // get the date and time for the filename
    time_t rawtime;
    struct tm * timeinfo;
    char datechar [80];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    
    strftime (datechar, 80, "%Y-%m-%d", timeinfo);
    
    return datechar;
}

string GetNextVideoFilename(string filenamePrefix,
    OpenCvStereoConfig configStruct, bool increment_number) {
    
    // format the number string
    char filenumber[100];
    sprintf(filenumber, "%02d",
        GetNextVideoNumber(configStruct, increment_number));
    
    string retstring = configStruct.videoSaveDir
                + "/" + filenamePrefix + "-"
                + GetDateSring()
                + "." + filenumber;
                
    if (!configStruct.usePGM) {
        return retstring + ".avi";
    } else {
        return retstring;
    }
}

/**
 * Sets up a video writer with the given filename
 *
 
 * @param frameSize size of the frames
 * @param filenamePrefix prefix for the new video filename
 * @param configStruct OpenCvStereoConfig structure
 *   for reading the directory things should be saved in
 *   and other data.
 * @param increment_number Default is true.  Set to false to
 *   not increment the video number.  Used for stereo vision
 *   systems if you want to have a left and right video with
 *   the same number (ie set to false for the second writer
 *   you initialize.)
 * @param is_color true if it is color
 *  @default false
 *
 * @retval VideoWriter object
 */
VideoWriter SetupVideoWriter(string filenamePrefix, Size frameSize, OpenCvStereoConfig configStruct, bool increment_number, bool is_color) {
    
    VideoWriter recorder;
    
    // note that this is only a local change to configStruct!
    configStruct.videoSaveDir = CheckOrCreateDirectory(configStruct.videoSaveDir);
    
    string filename = GetNextVideoFilename(filenamePrefix,
        configStruct, increment_number);
        
    char fourcc1 = configStruct.fourcc.at(0);
    char fourcc2 = configStruct.fourcc.at(1);
    char fourcc3 = configStruct.fourcc.at(2);
    char fourcc4 = configStruct.fourcc.at(3);
    
    recorder.open(filename, CV_FOURCC(fourcc1, fourcc2, fourcc3, fourcc4), 30, frameSize, is_color);
    if (!recorder.isOpened())
    {
        printf("VideoWriter failed to open!\n");
    } else {
        cout << endl << "Opened " << filename << endl;
    }
    
    return recorder;
}

/**
 * Checks to see if a directory exists.  If it does, returns, otherwise
 * creates the directory.
 * 
 * @param dir directory path
 * 
 * @retval directory path to use.  Will be the directory requested unless
 *  we failed to create that directory.
 * 
 */
string CheckOrCreateDirectory(string dir) {
    // check to make sure the directory exists (otherwise the
    // videowriter will seg fault)
    if (!boost::filesystem::exists(dir))
    {
        // directory does not exist, create it
        cout << "Warning: video save directory does not exist, creating it." << endl;
        
        if (boost::filesystem::create_directory(dir)) {
            cout << "Successfully created directory: " << dir << endl;
            return dir;
        } else {
            cout << "Warning: failed to create video save directory, attempting to proceed with the current directory." << endl;
            
            // note that this is only a local change to configStruct!
            return ".";
        }
    }
    return dir;
}

/**
 * Sets up the system to write .pgm files into a directory named in a nice way.
 * 
 * @param dirnamePrefix prefix for the save directory
 * @param configStruct OpenCvStereoConfig struct, used for configStruct.videoSaveDir
 * 
 * @retval directory path to use for saving images
 * 
 */
string SetupVideoWriterPGM(string dirnamePrefix, OpenCvStereoConfig configStruct, bool increment_number) {
    
    // make or use parent directory
    
    // note that this is only a local change to configStruct!
    configStruct.videoSaveDir = CheckOrCreateDirectory(configStruct.videoSaveDir);
    
    
    // make a new directory for a bunch of images
    string dirpath = GetNextVideoFilename(dirnamePrefix, configStruct, increment_number);
        
    
    if (boost::filesystem::create_directory(dirpath)) {
        cout << "Successfully created directory: " << dirpath << endl;
    } else {
        cout << "Warning: failed to create directory: " << dirpath << ", attempting to proceed with the default save directory." << endl;
        
        return configStruct.videoSaveDir;
    }
    return dirpath;
}

void InitBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool enable_gamma)
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
    
    if (enable_gamma) {
        dc1394_feature_set_power(camera1, DC1394_FEATURE_GAMMA, DC1394_ON);
    } else {
        dc1394_feature_set_power(camera1, DC1394_FEATURE_GAMMA, DC1394_OFF);
    }
    
    // for camera 2 (slave on brightness settings), set everything
    // to manual except framerate
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

/**
 * Gets video filesnames for loading the videos
 * 
 * @param left_video string to place  the left video filename into
 * @param right_video string to place the right video filename into
 * @param video_directory directory vidoe files are in
 * @param timestamp timestamp for extracting the date for the video filename
 * @param video_number requested video number
 * @param using_avi true if we should search for ".avi" videos, false if we should search for directories with pgm images.
 * 
 * @retval skip amount or -1 on failure to find the video.
 * 
 */
int GetVideoFilenamesForLoading(string &left_video, string &right_video, string video_directory, long long timestamp, int video_number, bool using_avi) {
    
    // if the directory has a trailing "/", zap it
    if (video_directory.back() == '/') {
        video_directory = video_directory.substr(0, video_directory.length() - 1);
    }
    
    // first, ensure the directory exists
    if (!boost::filesystem::exists(video_directory)) {
        cerr << "Warning: directory does not exist: " << video_directory << endl;
        return -1;
    }
    
    char tmbuf[64], buf[64];
    
    // convert the timestamp to a date
    struct tm *nowtm;
    time_t tv_sec = timestamp / 1000000.0;
    nowtm = localtime(&tv_sec);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d", nowtm);
    sprintf(buf, "%s", tmbuf);

    string datetime = buf;
    
    int skip_amount = MatchVideoFile(video_directory, datetime, using_avi, video_number);
    
    
    // format the video number as a two-decimal value
    char filenumber[100];
    sprintf(filenumber, "%02d", video_number);
    string video_number_str = filenumber;
    
    // now we have the full filename
    left_video = video_directory + "/videoL-skip-" + to_string(skip_amount) + "-" + datetime
        + "." + video_number_str;
        
    right_video = video_directory + "/videoR-skip-" + to_string(skip_amount) + "-" + datetime
        + "." + video_number_str;
        
    if (using_avi) {
        
        left_video += ".avi";
        right_video += ".avi";
    }
        
    return skip_amount;
}

/**
 * LoadVideoFileFromDir loads two video files from a directory, given a video
 * directory, a timestamp, and video number.
 * 
 * @param left_video_capture video capture to fill in for the left video
 * @param left_video_capture video capture to fill in for the right video
 * @param video_directory directory to search for video files
 * @param timestamp timestamp to extract the date out of
 * @param video_number video number to load
 * 
 * @retval frame skip amount, or -1 on failure.
 * 
 */
int LoadVideoFileFromDir(VideoCapture *left_video_capture, VideoCapture *right_video_capture, string video_directory, long long timestamp, int video_number) {
    
    string left_video, right_video;
    
    int skip_amount = GetVideoFilenamesForLoading(left_video, right_video, video_directory, timestamp, video_number, true);
    
    if (skip_amount < 0) {
        cout << "Error getting video filesnames, not loading videos." << endl;
        return -1;
    }
    
    
    // attempt to create video capture objects
    
    if (!left_video_capture) {
        left_video_capture = new VideoCapture();
    } else {
        left_video_capture->release();
    }
    
    if (!right_video_capture) {
        right_video_capture = new VideoCapture();
    } else {
        right_video_capture->release();
    }
    
    cout << endl << "Loading:" << endl << "\t" << left_video << endl << "\t" << right_video << endl;
    
    left_video_capture->open(left_video);
    if (!left_video_capture->isOpened()) {
        cerr << "Error: failed to load " << left_video << endl;
        return -1;
    }
    
    right_video_capture->open(right_video);
    if (!right_video_capture->isOpened()) {
        cerr << "Error: failed to load " << right_video << endl;
        return -1;
    }
    
    
    return skip_amount;
    
    
}

int InitPGMLoading(string video_directory, long long timestamp, int video_number) {
    
    string left_video, right_video;
    
    int skip_amount = GetVideoFilenamesForLoading(left_video, right_video, video_directory, timestamp, video_number, false);
    
    if (skip_amount < 0) {
        cout << "Error getting video filesnames, not loading videos." << endl;
        return -1;
    }
    
    // TODO TODO TODO
    
    
    
}


/**
 * Send an image over LCM (useful for viewing in a
 * headless configuration
 *
 * @param lcm already initialized lcm object
 * @param image the image to send
 * @param channel channel name to send over
 *
 */
void SendImageOverLcm(lcm_t* lcm, string channel, Mat image) {
    
    // create LCM message
    bot_core_image_t msg;
    
    msg.utime = getTimestampNow();
    
    msg.width = image.cols;
    msg.height = image.rows;
    msg.row_stride = image.cols;
    
    msg.pixelformat = 1497715271; // see bot_core_iamge_t.lcm --> PIXEL_FORMAT_GRAY; // TODO: detect this
    
    
    if (image.isContinuous()) {
    
        msg.data = image.ptr();
        
        msg.size = image.cols * image.rows;
        
        msg.nmetadata = 0;
        
        // send the image over lcm
        bot_core_image_t_publish(lcm, channel.c_str(), &msg);
    } else {
        cout << "Image not continuous. LCM transport not implemented." << endl;
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
 * @param color color to draw the boxes
 */
void Draw3DPointsOnImage(Mat camera_image, vector<Point3f> *points_list_in, Mat cam_mat_m, Mat cam_mat_d, Scalar color)
{
    vector<Point3f> &points_list = *points_list_in;
    
    if (points_list.size() <= 0)
    {
        cout << "Draw3DPointsOnimage: zero sized points list" << endl;
        return;
    }
    
    vector<Point2f> img_points_list;

    projectPoints(points_list, Mat::zeros(3, 1, CV_32F), Mat::zeros(3, 1, CV_32F), cam_mat_m, cam_mat_d, img_points_list);
    
    // now draw the points onto the image
    for (int i=0; i<int(img_points_list.size()); i++)
    {
        //rectangle(camera_image, Point(img_points_list[i].x - 2, img_points_list[i].y - 2),
    //        Point(img_points_list[i].x + 2, img_points_list[i].y + 2), color, CV_FILLED);
            
        rectangle(camera_image, Point(img_points_list[i].x - 4, img_points_list[i].y - 4),
            Point(img_points_list[i].x + 4, img_points_list[i].y + 4), color, CV_FILLED);
    }
    
}




void GetNextVideoFrameAvi(VideoCapture left_video_capture, VideoCapture right_video_capture, Mat matL, Mat matR,
    int file_frame_number, int file_frame_skip) {
        
    Mat matL_file, matR_file;

    // make sure we don't run off the end of the video file and crash
    if (file_frame_number - file_frame_skip
        >= left_video_capture->get(CV_CAP_PROP_FRAME_COUNT)) {
            
        file_frame_number = left_video_capture->get(CV_CAP_PROP_FRAME_COUNT) - 1 + file_frame_skip;
    }

    // make sure we don't try to play before the file starts
    if (file_frame_number - file_frame_skip < 0) {
        file_frame_number = file_frame_skip;
    }

    left_video_capture->set(CV_CAP_PROP_POS_FRAMES, file_frame_number - file_frame_skip);
    right_video_capture->set(CV_CAP_PROP_POS_FRAMES, file_frame_number - file_frame_skip);

    (*left_video_capture) >> matL_file;
    (*right_video_capture) >> matR_file;

    // convert from a 3 channel array to a one channel array
    cvtColor(matL_file, matL, CV_BGR2GRAY);
    cvtColor(matR_file, matR, CV_BGR2GRAY);

}

void GetNextVideoFramePgm(string left_pgm_dir, string right_pgm_dir, Mat matL, Mat matR,
    int file_frame_number, int file_frame_skip) {
    
    // format the frame number as a 5-character number
    
    
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
        
        //cout << "exposure: " << exposure_value << " brightness: " << brightness_value << endl;
        
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
    
    
    
    cout << endl << dc1394_feature_get_string(features.feature[0].id) << ": " << features.feature[0].value << "/" << features2.feature[0].value << endl;

    
    // set exposure
    cout << endl << dc1394_feature_get_string(features.feature[1].id) << ": " << features.feature[1].value << "/" << features2.feature[1].value << endl;

    
    // set gamma
    cout << endl << dc1394_feature_get_string(features.feature[6].id) << ": " << features.feature[6].value << "/" << features2.feature[6].value << endl;

    
    // set shutter
    cout << endl << dc1394_feature_get_string(features.feature[7].id) << ": " << features.feature[7].value << "/" << features2.feature[7].value << endl;
    
    // set gain
   cout << endl << dc1394_feature_get_string(features.feature[8].id) << ": " << features.feature[8].value << "/" << features2.feature[8].value << endl;
    
    //cout << "---------------------------------------" << endl;
    //dc1394_feature_print_all(&features, stdout);
    //cout << "222222222222222222222222222222222222222222222" << endl;
    //dc1394_feature_print_all(&features2, stdout);
    //dc1394_feature_get_absolute_value(camera1, DC1394_FEATURE_BRIGHTNESS, &brightnessVal);
    
    //cout << endl << "brightness: " << brightnessVal << endl;
    #endif
}

