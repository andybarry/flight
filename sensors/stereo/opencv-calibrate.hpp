/*
 * Program that grabs stereo pairs from Point Grey
 * Firefly MV USB cameras and captures images for
 * calibration.
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */

#ifndef OPENCV_CALIBRATE
#define OPENCV_CALIBRATE

#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include "../../externals/ConciseArgs.hpp"

#include "opencv-stereo-util.hpp"

using namespace cv;

extern "C"
{
    #include <stdio.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <inttypes.h>

    #include <glib.h>
    #include <dc1394/dc1394.h>

    #include "camera.h"
    #include "utils.h"
    #include "opencvutils.h"
}

#define BRIGHTNESS_VALUE 78
#define EXPOSURE_VALUE 128

#define MAX_FRAMES 2000

#define CHESS_X 9
#define CHESS_Y 6

#endif
