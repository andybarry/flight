/*
 * Utility functions for opencv-stereo
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */
 
#ifndef OPENCV_STEREO_UTIL
#define OPENCV_STEREO_UTIL

#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/opencv.hpp"

#include <sys/time.h>

#include <boost/filesystem.hpp>

#include <string>
#include <glib.h> // for configuration files

#include "lcmtypes/bot_core_image_t.h" // from libbot for images over LCM

extern "C"
{
    #include <stdio.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <inttypes.h>

    #include <dc1394/dc1394.h>

    #include "camera.h"
    #include "utils.h"
}


using namespace std;
using namespace cv;

struct OpenCvStereoConfig
{
    uint64 guidLeft;
    uint64 guidRight;
    
    string lcmUrl;
    string stereoControlChannel;
    string calibrationDir;
    string videoSaveDir;
    string fourcc;
    
    string stereo_replay_channel;
    string baro_airspeed_channel;
    string pose_channel;
    string gps_channel;
    string battery_status_channel;
    string servo_out_channel;
    string optotrak_channel;
    
    int disparity;
    int infiniteDisparity;
    int interestOperatorLimit;
    int blockSize;
    int sadThreshold;
    
    float interestOperatorDivisor;
    
};

struct OpenCvStereoCalibration
{
    Mat mx1fp;
    Mat mx2fp;
    Mat qMat;
};

Mat GetFrameFormat7(dc1394camera_t *camera);

int64_t getTimestampNow();

bool ParseConfigFile(string configFile, OpenCvStereoConfig *configStruct);

bool LoadCalibration(string calibrationDir, OpenCvStereoCalibration *stereoCalibration);

int LoadVideoFileFromDir(VideoCapture *left_video_capture, VideoCapture *right_video_capture, string video_directory, long timestamp, int video_number);

void StopCapture(dc1394_t *dcContext, dc1394camera_t *camera);

int GetNextVideoNumber(OpenCvStereoConfig configStruct,
    bool increment_number = true);
    
int MatchVideoFile(string directory, string datestr, int match_number = -1);

int GetSkipNumber(string filename);

string GetDateSring();

string GetNextVideoFilename(string filenamePrefix,
    OpenCvStereoConfig configStruct, bool increment_number = true);

VideoWriter SetupVideoWriter(string filename, Size frameSize, OpenCvStereoConfig configStruct, bool increment_number = true);

void InitBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool enable_gamma = false);

void MatchBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool complete_set = false, int force_brightness = -1, int force_exposure = -1);

void SendImageOverLcm(lcm_t* lcm, string channel, Mat image);

#endif
