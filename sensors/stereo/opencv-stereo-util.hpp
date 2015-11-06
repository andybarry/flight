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

#include "../../LCM/lcmt_stereo.h"
#include "../../utils/utils/RealtimeUtils.hpp"



extern "C"
{
    #include <stdio.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <inttypes.h>

    #include <dc1394/dc1394.h>

    #include <camera.h>
    #include <utils.h>

    #include "../../externals/jpeg-utils/jpeg-utils.h"
}


using namespace cv;

struct OpenCvStereoConfig
{
    uint64 guidLeft;
    uint64 guidRight;

    string lcmUrl;
    string stereoControlChannel;
    string calibrationDir;
    float calibrationUnitConversion;
    int lastValidPixelRow;
    string videoSaveDir;
    string fourcc;

    bool usePGM;

    string stereo_replay_channel;
    string baro_airspeed_channel;
    string pose_channel;
    string gps_channel;
    string battery_status_channel;
    string servo_out_channel;
    string optotrak_channel;
    string cpu_info_channel1;
    string cpu_info_channel2;
    string cpu_info_channel3;

    string log_size_channel1;
    string log_size_channel2;
    string log_size_channel3;


    int disparity;
    int infiniteDisparity;
    int interestOperatorLimit;
    int blockSize;
    int sadThreshold;
    float horizontalInvarianceMultiplier;

    int displayOffsetX;
    int displayOffsetY;

};

struct OpenCvStereoCalibration
{
    Mat mx1fp;
    Mat mx2fp;
    Mat qMat;

    Mat M1;
    Mat D1;
    Mat R1;
    Mat P1;

    Mat M2;
    Mat D2;
    Mat R2;
    Mat P2;
};

Mat GetFrameFormat7(dc1394camera_t *camera);

void FlushCameraBuffer(dc1394camera_t *camera);

int64_t getTimestampNow();

bool ParseConfigFile(string configFile, OpenCvStereoConfig *configStruct);

bool LoadCalibration(string calibrationDir, OpenCvStereoCalibration *stereoCalibration);

void StopCapture(dc1394_t *dcContext, dc1394camera_t *camera);


void InitBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool enable_gamma = false);

void MatchBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2, bool complete_set = false, int force_brightness = -1, int force_exposure = -1);

void SendImageOverLcm(lcm_t* lcm, string channel, Mat image, int compression_quality = 80);

void Get3DPointsFromStereoMsg(const lcmt_stereo *msg, vector<Point3f> *points_out);

void Draw3DPointsOnImage(Mat camera_image, vector<Point3f> *points_list_in, Mat cam_mat_m, Mat cam_mat_d, Mat cam_mat_r, Scalar outline_color = 128, Scalar inside_color = 255, Point2d box_top = Point2d(-1, -1), Point2d box_bottom = Point2d(-1, -1), vector<int> *points_in_box = NULL, float min_z = 0, float max_z = 0, int box_size = 4);

int GetDisparityForDistance(double distance, const OpenCvStereoCalibration &calibration, int *inf_disparity = NULL);

#endif
