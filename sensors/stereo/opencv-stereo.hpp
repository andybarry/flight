/*
 * Program that grabs stereo pairs from Point Grey
 * Firefly MV USB cameras, perfroms the single-disparity
 * stereo algorithm on them, and publishes the results
 * to LCM.
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */
 
#ifndef OPENCV_STEREO
#define OPENCV_STEREO

#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include <lcm/lcm.h>
#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_control.h"
#include "../../LCM/lcmt_baro_airspeed.h"
#include "../../LCM/lcmt_battery_status.h"
#include "../../LCM/lcmt_deltawing_u.h"
#include "../../LCM/mav_gps_data_t.h"
#include "../../LCM/mav_pose_t.h"


#include "../../LCM/lcmt_stereo_control.h"

#include "../../externals/ConciseArgs.hpp"



#include <signal.h>
#include <stdlib.h>
#include <stdio.h>


//#include <libusb.h> // for USB reset

#include "opencv-stereo-util.hpp"
#include "barrymoore.hpp"
#include "hud.hpp"

using namespace std;
using namespace cv;

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

#define MAX_HIT_POINTS 320*240/25 // this is the most hits we can get with our setup TODO: fixme for the correct framesize

#define RINGBUFFER_SIZE (120*50) // number of seconds to allocate for recording * framerate


#define USE_IMAGE 0 // set to 1 to use left.jpg and right.jpg as test images

#define MATCH_BRIGHTNESS_EVERY_N_FRAMES 10

struct RemapState
{
    Mat inputImage;
    Mat outputImage;
    Mat map1;
    Mat map2;
    int flags;
};

bool NonBlockingLcm(lcm_t *lcm);

bool ResetPointGreyCameras();

void StartRecording();

void WriteVideo();

void onMouse( int event, int x, int y, int, void* );
void DrawLines(Mat leftImg, Mat rightImg, Mat stereoImg, int lineX, int lineY, int disparity);

void stereo_replay_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user);

void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user);

void battery_status_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_battery_status *msg, void *user);

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user);

void mav_gps_data_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_gps_data_t *msg, void *user);

void mav_pose_t_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user);


#endif
