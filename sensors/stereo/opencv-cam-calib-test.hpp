#ifndef OPENCV_SINGLE_CALIB_TEST_HPP
#define OPENCV_SINGLE_CALIB_TEST_HPP

#include "opencv-stereo-util.hpp"

#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include "../../externals/ConciseArgs.hpp"

void control_c_handler(int s);
void CleanUp();

#endif
