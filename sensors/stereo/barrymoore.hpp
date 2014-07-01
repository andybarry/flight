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
#include <math.h>
#include <random> // for debug random generator

#ifdef USE_NEON
#include <arm_neon.h>
#endif // USE_NEON

#define NUM_THREADS 8
//#define NUM_REMAP_THREADS 8

using namespace cv;
using namespace std;

enum ThreadWorkType { REMAP, INTEREST_OP, STEREO };

struct BarryMooreState
{
    int disparity;
    int zero_dist_disparity;
    int sobelLimit;
    int blockSize;
    int sadThreshold;

    int lastValidPixelRow;

    // a parameter added to the SAD/sobel multiplier in the denominator
    float interestOperatorMultiplier;
    float interestOperatorMultiplierHorizontalInvariance;

    Mat mapxL;

    Mat mapxR;

    Mat Q;

    bool show_display;

    float random_results;

    float debugJ, debugI, debugDisparity;

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
    Mat left_image;
    Mat right_image;

    Mat sub_remapped_left_image;
    Mat sub_remapped_right_image;

    Mat sub_laplacian_left;
    Mat sub_laplacian_right;

    Mat submapxL;
    Mat submapxR;
};

struct InterestOpState {
    Mat left_image;
    Mat right_image;

    Mat sub_laplacian_left;
    Mat sub_laplacian_right;

    int row_start;
    int row_end;
};


class BarryMoore {
    private:
        void RunStereoBarryMoore(BarryMooreStateThreaded *statet);

        void RunRemapping(RemapThreadState *remap_state);

        void RunInterestOp(InterestOpState *interest_state);

        int GetSAD(Mat leftImage, Mat rightImage, Mat laplacianL, Mat laplacianR, int pxX, int pxY, BarryMooreState state);

        bool CheckHorizontalInvariance(Mat leftImage, Mat rightImage, Mat sobelL, Mat sobelR, int pxX, int pxY, BarryMooreState state);

        void StartWorkerThread(int i, ThreadWorkType work_type);
        void SyncWorkerThreads();

        // this must be static so the threading won't have to
        // deal with the implicit 'this' variable
        static void* WorkerThread(void *x);

        int RoundUp(int numToRound, int multiple);

        pthread_t worker_pool_[NUM_THREADS+1];
        ThreadWorkType work_type_[NUM_THREADS+1];

        bool has_new_data_[NUM_THREADS+1];

        BarryMooreStateThreaded thread_states_[NUM_THREADS+1];
        RemapThreadState remap_thread_states_[NUM_THREADS+1];
        InterestOpState interest_op_states_[NUM_THREADS+1];

        mutex data_mutexes_[NUM_THREADS+1];

        condition_variable cv_new_data_[NUM_THREADS+1];
        condition_variable cv_thread_finish_[NUM_THREADS+1];

        unique_lock<mutex> lockers_[NUM_THREADS+1];


    public:
        BarryMoore();

        void ProcessImages(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<uchar> *pointColors, cv::vector<Point3i> *pointVector2d, BarryMooreState state);

        ThreadWorkType GetWorkType(int i) { return work_type_[i]; }

        BarryMooreStateThreaded* GetThreadedState(int i) {
            return &(thread_states_[i]);
        }

        bool GetHasNewData(int i) { return has_new_data_[i]; }
        void SetHasNewData(int i, bool val) { has_new_data_[i] = val; }

        RemapThreadState* GetRemapState(int i) { return &(remap_thread_states_[i]); }

        InterestOpState* GetInterestOpState(int i) { return &(interest_op_states_[i]); }

};

struct BarryMooreThreadStarter {
    int thread_number;
    mutex *data_mutex;
    condition_variable *cv_new_data;
    condition_variable *cv_thread_finish;
    bool *is_working;

    BarryMoore *parent;
};


#endif
