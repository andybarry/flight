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
#include <mutex>
#include <condition_variable>

#define NUM_THREADS 2
//#define NUM_REMAP_THREADS 8

using namespace cv;
using namespace std;

struct BarryMooreState
{
    int disparity;
    int zero_dist_disparity;
    int sobelLimit;
    int blockSize;
    int sadThreshold;
    
    // a parameter added to the SAD/sobel division in the denominator
    float sobelAdd;
    
    Mat mapxL;
    
    Mat mapxR;
    
    Mat Q;
    
    bool show_display;
    
    //cv::vector<Point3f> *localHitPoints; // this is an array of cv::vector<Point3f>'s
};

struct BarryMooreStateThreaded {
    BarryMooreState state;
    
    Mat remapped_left;
    Mat remapped_right;
    
    Mat laplacian_left;
    Mat laplacian_right;
    
    cv::vector<Point3f> *pointVector3d;
    cv::vector<Point3i> *pointVector2d;
    cv::vector<uchar> *pointColors;

    int row_start;
    int row_end;
    
    
};

struct RemapThreadState {
    Mat leftImage;
    Mat rightImage;
    
    Mat sub_remapped_left_image;
    Mat sub_remapped_right_image;
    
    Mat sub_laplacian_left;
    Mat sub_laplacian_right;
    
    Mat submapxL;
    Mat submapxR;
};

class BarryMoore {
    private:
        void RunStereoBarryMoore(BarryMooreStateThreaded *statet);

        void RunRemapping(RemapThreadState *remap_state);

        int GetSAD(Mat leftImage, Mat rightImage, Mat laplacianL, Mat laplacianR, int pxX, int pxY, BarryMooreState state);

        bool CheckHorizontalInvariance(Mat leftImage, Mat rightImage, Mat sobelL, Mat sobelR, int pxX, int pxY, BarryMooreState state);
        
        // this must be static so the threading won't have to
        // deal with the implicit 'this' variable
        static void* WorkerThread(void *x);
        
        pthread_t worker_pool_[NUM_THREADS+1];
        bool is_remapping_[NUM_THREADS+1];
        
        BarryMooreStateThreaded thread_states_[NUM_THREADS+1];
        RemapThreadState remap_thread_states_[NUM_THREADS+1];
        
        mutex running_mutexes_[NUM_THREADS+1];
        mutex data_mutexes_[NUM_THREADS+1];
        
        
    public:
        BarryMoore();
        
        void ProcessImages(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<uchar> *pointColors, cv::vector<Point3i> *pointVector2d, BarryMooreState state);
        
        bool GetIsRemapping(int i) { return is_remapping_[i]; }
        BarryMooreStateThreaded* GetThreadedState(int i) {
            return &(thread_states_[i]);
        }
        
        RemapThreadState* GetRemapState(int i) { return &(remap_thread_states_[i]); }
        
};

struct BarryMooreThreadStarter {
    int thread_number;
    mutex *thread_running_mutex;
    mutex *data_mutex;
    condition_variable *ready_cv;
    
    BarryMoore *parent;
};

 
#endif
