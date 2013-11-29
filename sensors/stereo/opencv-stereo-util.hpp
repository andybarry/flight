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
    
    int disparity;
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

void StopCapture(dc1394_t *dcContext, dc1394camera_t *camera);

int GetNextVideoNumber(OpenCvStereoConfig configStruct,
    bool increment_number = true);

string GetDateSring();

string GetNextVideoFilename(string filenamePrefix,
    OpenCvStereoConfig configStruct, bool increment_number = true);

VideoWriter SetupVideoWriter(string filename, Size frameSize, OpenCvStereoConfig configStruct, bool increment_number = true);

void InitBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2);

void MatchBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool complete_set = false, int force_brightness = -1, int force_exposure = -1);

#endif
