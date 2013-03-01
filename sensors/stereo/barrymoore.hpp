/*
 * Implements the fast, single-disparity stereo alogrithm.
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */
 
#ifndef BARRYMOORE_H
#define BARRYMOORE_H
 
#include "opencv2/opencv.hpp"
#include <cv.h>
#include <iostream>

#define SHOW_DISPLAY 0

#define NUM_THREADS 8


using namespace cv;
using namespace std;

struct BarryMooreState
{
    int disparity;
    int sobelLimit;
    int blockSize;
    int sadThreshold;
    
    Mat mapxL;
    
    Mat mapxR;
    
    Mat Q;
    
    //cv::vector<Point3f> *localHitPoints; // this is an array of cv::vector<Point3f>'s
};

struct BarryMooreStateThreaded
{
    BarryMooreState state;
    
    Mat leftImage;
    Mat rightImage;
    cv::vector<Point3f> *pointVector3d;
    cv::vector<Point> *pointVector2d;
    //cv::vector<Point3f> *localHitPoints;
    
    int rowOffset;
    
    Mat submapxL;
    
    Mat submapxR;
    
};

void StereoBarryMoore(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<Point> *pointVector2d, BarryMooreState state);

void* StereoBarryMooreThreaded(void *statet);

int GetSAD(Mat leftImage, Mat rightImage, Mat sobelL, Mat sobelR, int pxX, int pxY, BarryMooreState state);

 
#endif
