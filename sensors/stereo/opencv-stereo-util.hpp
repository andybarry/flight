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

#include <string>
#include <glib.h> // for configuration files


using namespace std;
using namespace cv;

struct OpenCvStereoConfig
{
    uint64 guidLeft;
    uint64 guidRight;
    
    string lcmUrl;
    string stereoControlChannel;
    string calibrationDir;
};

struct OpenCvStereoCalibration
{
    Mat mx1fp;
    Mat mx2fp;
    Mat qMat;
};

bool ParseConfigFile(string configFile, OpenCvStereoConfig *configStruct);

bool LoadCalibration(string calibrationDir, OpenCvStereoCalibration *stereoCalibration);


#endif
