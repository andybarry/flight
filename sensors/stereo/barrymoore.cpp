/*
 * Implements the fast, single-disparity stereo alogrithm.
 *
 * Copyright 2013, Andrew Barry <abarry@csail.mit.edu>
 *
 */
 
#include "barrymoore.hpp"
#include <pthread.h>

// if USE_SAFTEY_CHECKS is 1, GetSAD will try to make sure
// that it will do the right thing even if you ask it for pixel
// values near the edges of images.  Set to 0 for a small speedup.
#define USE_SAFTEY_CHECKS 0

#define INVARIANCE_CHECK_VERT_OFFSET_MIN (-8)
#define INVARIANCE_CHECK_VERT_OFFSET_MAX 8
#define INVARIANCE_CHECK_VERT_OFFSET_INCREMENT 2

#define INVARIANCE_CHECK_HORZ_OFFSET_MIN (-2)
#define INVARIANCE_CHECK_HORZ_OFFSET_MAX 2

BarryMooreThreadStarter thread_starter[NUM_THREADS+1];


BarryMoore::BarryMoore() {
    // init worker threads
    
    
    
    for (int i = 0; i < NUM_THREADS; i++) {
        // start all the worker threads
        
        SetHasNewData(i, false);
        data_mutexes_[i].lock();
        
        thread_starter[i].thread_number = i;
        thread_starter[i].data_mutex = &(data_mutexes_[i]);
        thread_starter[i].cv_new_data = &(cv_new_data_[i]);
        thread_starter[i].cv_thread_finish = &(cv_thread_finish_[i]);
        thread_starter[i].parent = this;
        
        // start the thread
        pthread_create(&(worker_pool_[i]), NULL, WorkerThread, &thread_starter[i]);
        
    }
}

void* BarryMoore::WorkerThread(void *x) {

    BarryMooreThreadStarter *statet = (BarryMooreThreadStarter*) x;

    mutex *data_mutex = statet->data_mutex;
    int thread_number = statet->thread_number;
    condition_variable *cv_new_data = statet->cv_new_data;
    condition_variable *cv_thread_finish = statet->cv_thread_finish;
    
    BarryMoore *parent = statet->parent;

    

    while (true) {
        
        // wait on the condition variable for
        // "has new data"
        mutex temp;
        unique_lock<mutex> locker(temp);
        
        cout << "[thread " << thread_number << "] waiting" << endl;
        while (parent->GetHasNewData(thread_number) == false) {
            cv_new_data->wait(locker);
        }
        
        // if we don't already have the mutex, get it
        data_mutex->lock();
        
        cout << "[thread " << thread_number << "] going" << endl;
        
        parent->SetHasNewData(thread_number, false);
        
        
        
        // if we're here, there's work to be done
        cout << "[thread " << thread_number << "] running" << endl;
            
        // see if that work is remapping or stereo processing
        switch (parent->GetWorkType(thread_number)) {
            case REMAP:
                parent->RunRemapping(parent->GetRemapState(thread_number));
                break;
                
            case INTEREST_OP:
                parent->RunInterestOp(parent->GetInterestOpState(thread_number));
                break;
                                
            case STEREO:
                parent->RunStereoBarryMoore(parent->GetThreadedState(thread_number));
                break;
                
            default:
                cerr << "Warning: unknown thread work type." << endl;
                break;
        }
        
        // done, signal the waiting main thread

        cv_thread_finish->notify_all();
        
        data_mutex->unlock();

        cout << "[thread " << thread_number << "] done" << endl;
        
    }
    
    return NULL;
}

/**
 * Runs the fast, single-disparity stereo algorithm.  Returns a
 * vector of points where it found disparity matches in the image.
 *
 * @param _leftImage left camera image as a CV_8UC1
 * @param _rightImage right camera image as a CV_8UC1
 * @param hitVector a cv::vector that we will populate with cv::Point()s
 * @param state set of configuration parameters for the function.
 *      You can change these on each run of the function if you'd like.
 */
void BarryMoore::ProcessImages(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<uchar> *pointColors, cv::vector<Point3i> *pointVector2d, BarryMooreState state) {
    
    cout << "[main] entering process images" << endl;
    
    Mat leftImage = _leftImage.getMat();
    Mat rightImage = _rightImage.getMat();
    
    // make sure that the inputs are of the right type
    CV_Assert(leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1);
    
    // we want to use the sum-of-absolute-differences (SAD) algorithm
    // on a single disparity
    
    // split things up so we can parallelize
    int rows = leftImage.rows;
    
    // first parallelize remaping
    
    // we split these arrays up and send them into each
    // thread so at the end, each thread has written to the
    // appropriate spot in the array
    Mat remapped_left(state.mapxL.rows, state.mapxL.cols, leftImage.depth());
    Mat remapped_right(state.mapxR.rows, state.mapxR.cols, rightImage.depth());
    
    
    
    for (int i = 0; i < NUM_THREADS; i++) {
        
        int start = rows/NUM_THREADS*i;
        int end = rows/NUM_THREADS*(i+1);
        
        remap_thread_states_[i].left_image = leftImage;
        remap_thread_states_[i].right_image = rightImage;
        
        // send in a subset of the map
        remap_thread_states_[i].submapxL = state.mapxL.rowRange(start, end);
        remap_thread_states_[i].submapxR = state.mapxR.rowRange(start, end);
        
        // send in subsets of the arrays to be filled in
        remap_thread_states_[i].sub_remapped_left_image = remapped_left.rowRange(start, end);
        
        remap_thread_states_[i].sub_remapped_right_image = remapped_right.rowRange(start, end);
        
        StartWorkerThread(i, REMAP);
        
    }
    cout << "[main] all remap threads started" << endl;
    
    
    // wait for all remapping threads to finish
    SyncWorkerThreads();
    
    cout << "[main] all remap threads finished" << endl;
    
    Mat laplacian_left(remapped_left.rows, remapped_left.cols, remapped_left.depth());
    Mat laplacian_right(remapped_right.rows, remapped_right.cols, remapped_right.depth());
    
    for (int i = 0; i < NUM_THREADS; i++) {
        
        int start = rows/NUM_THREADS*i;
        int end = rows/NUM_THREADS*(i+1);
        
        interest_op_states_[i].left_image = remapped_left;
        interest_op_states_[i].right_image = remapped_right;
        
        // send in subsets of the arrays to be filled in
        
        interest_op_states_[i].sub_laplacian_left = laplacian_left.rowRange(start, end);
        interest_op_states_[i].sub_laplacian_right = laplacian_right.rowRange(start, end);
        
        interest_op_states_[i].row_start = start;
        interest_op_states_[i].row_end = end;
        
        
        StartWorkerThread(i, INTEREST_OP);
        
    }
    
    SyncWorkerThreads();
    
    // now we have fully remapped both images
    imshow("Left Block", remapped_right);
    imshow("Right Block", laplacian_right);
    
    cout << "[main] imshow2 ok" << endl;
    
    cv::vector<Point3f> pointVector3dArray[NUM_THREADS+1];
    cv::vector<Point3i> pointVector2dArray[NUM_THREADS+1];
    cv::vector<uchar> pointColorsArray[NUM_THREADS+1];
    
    cout << "[main] firing worker threads..." << endl;
    for (int i=0;i<NUM_THREADS;i++)
    {
        
        thread_states_[i].state = state;
        
        int start = rows/NUM_THREADS*i;
        int end = rows/NUM_THREADS*(i+1);
        
        // send in the whole image because each thread needs
        // the entire image to do its small remapping job
        thread_states_[i].remapped_left = remapped_left; 
        thread_states_[i].remapped_right = remapped_right;
        
        thread_states_[i].laplacian_left = laplacian_left;
        thread_states_[i].laplacian_right = laplacian_right;
        
        thread_states_[i].pointVector3d = &pointVector3dArray[i];
        thread_states_[i].pointVector2d = &pointVector2dArray[i];
        thread_states_[i].pointColors = &pointColorsArray[i];
        thread_states_[i].row_start = start;
        thread_states_[i].row_end = end;
        
        
        
        // fire the worker thread
        StartWorkerThread(i, STEREO);
    }
    
    // wait for all the threads to come back
    SyncWorkerThreads();
    
    int numPoints = 0;
    // compute the required size of our return vector
    // this prevents multiple memory allocations
    for (int i=0;i<NUM_THREADS;i++)
    {
        numPoints += pointVector3dArray[i].size();
    }
    pointVector3d->reserve(numPoints);
    pointColors->reserve(numPoints);
    
    // combine the hit vectors
    for (int i=0;i<NUM_THREADS;i++)
    {
        pointVector3d->insert( pointVector3d->end(), pointVector3dArray[i].begin(), pointVector3dArray[i].end() );
        
        pointColors->insert( pointColors->end(), pointColorsArray[i].begin(), pointColorsArray[i].end() );
        
        if (state.show_display)
        {
            pointVector2d->insert( pointVector2d->end(), pointVector2dArray[i].begin(), pointVector2dArray[i].end() );
        }
    }
    
}

void BarryMoore::StartWorkerThread(int i, ThreadWorkType work_type) {

    
    work_type_[i] = work_type;
    
    SetHasNewData(i, true);
    
    data_mutexes_[i].unlock();
    cv_new_data_[i].notify_all();
    
}

void BarryMoore::SyncWorkerThreads() {

    for (int i=0;i<NUM_THREADS;i++)
    {
        mutex temp;
        unique_lock<mutex> locker(temp);
        
        while (GetHasNewData(i) == true) {
        
            // wait for the thread to finish
            cv_thread_finish_[i].wait(locker);
        }
        data_mutexes_[i].lock();
    }
}

/**
 * Function (for running in a thread) that remaps images
 *
 */
void BarryMoore::RunRemapping(RemapThreadState *remap_state) {
    
    // remap this part of the image
    
    remap(remap_state->left_image, remap_state->sub_remapped_left_image, remap_state->submapxL, Mat(), INTER_NEAREST);
    remap(remap_state->right_image, remap_state->sub_remapped_right_image, remap_state->submapxR, Mat(), INTER_NEAREST);
    
    
    
}

void BarryMoore::RunInterestOp(InterestOpState *interest_state) {

    // apply interest operator
    Laplacian(interest_state->left_image.rowRange(interest_state->row_start, interest_state->row_end), interest_state->sub_laplacian_left, -1, 3, 1, 0, BORDER_DEFAULT);
    
    Laplacian(interest_state->right_image.rowRange(interest_state->row_start, interest_state->row_end), interest_state->sub_laplacian_right, -1, 3, 1, 0, BORDER_DEFAULT);
    
}

/**
 * Function that actually does the work for the BarryMoore algorithm.
 *
 * @param statet all the parameters are set
 *          as a BarryMooreStateThreaded struct.
 *
 * @retval will always be NULL since the real values are passed
 *      back in the vector that is in statet.
 */
void BarryMoore::RunStereoBarryMoore(BarryMooreStateThreaded *statet)
{

    Mat leftImage = statet->remapped_left;
    Mat rightImage = statet->remapped_right;
    Mat laplacian_left = statet->laplacian_left;
    Mat laplacian_right = statet->laplacian_right;
    
    cv::vector<Point3f> *pointVector3d = statet->pointVector3d;
    cv::vector<Point3i> *pointVector2d = statet->pointVector2d;
    cv::vector<uchar> *pointColors = statet->pointColors;
    
    int row_start = statet->row_start;
    int row_end = statet->row_end;
    
    BarryMooreState state = statet->state;

    // we will do this by looping through every block in the left image
    // (defined by blockSize) and checking for a matching value on
    // the right image

    cv::vector<Point3f> localHitPoints;

    int blockSize = state.blockSize;
    int disparity = state.disparity;
    int sadThreshold = state.sadThreshold;
    
    int startJ = 0;
    int stopJ = leftImage.cols - (disparity + blockSize);
    if (disparity < 0)
    {
        startJ = -disparity;
        stopJ = leftImage.cols - blockSize;
    }
    
    int hitCounter = 0;
    
    for (int i=row_start; i < row_end; i+=blockSize)
    {
        for (int j=startJ; j < stopJ; j+=blockSize)
        {
            // get the sum of absolute differences for this location
            // on both images
            int sad = GetSAD(leftImage, rightImage, laplacian_left, laplacian_right, j, i, state);
            // check to see if the SAD is below the threshold,
            // indicating a hit
            if (sad < sadThreshold && sad >= 0)
            {
                // got a hit
                
                // now check for horizontal invariance
                // (ie check for parts of the image that look the same as this
                // which would indicate that this might be a false-positive)
    
                if (CheckHorizontalInvariance(leftImage, rightImage, laplacian_left, laplacian_right, j, i, state) == false) {
                
                    // add it to the vector of matches
                    // don't forget to offset it by the blockSize,
                    // so we match the center of the block instead
                    // of the top left corner
                    localHitPoints.push_back(Point3f(j+blockSize/2.0, i+blockSize/2.0, disparity));
                    
                    uchar pxL = leftImage.at<uchar>(i,j);
                    pointColors->push_back(pxL); // TODO: this is the corner of the box, not the center
                    
                    hitCounter ++;
                    
                    if (state.show_display)
                    {
                        pointVector2d->push_back(Point3i(j, i, sad));
                    }
                } // check horizontal invariance
            }
        }
    }
    
    // now we have an array of hits -- transform them to 3d points
    if (hitCounter > 0)
    {
        perspectiveTransform(localHitPoints, *pointVector3d, state.Q);
    }
    
}


/**
 * Get the sum of absolute differences for a specific pixel location and disparity
 *
 * @param leftImage left image
 * @param rightImage right image
 * @param laplacianL laplacian-fitlered left image
 * @param laplacianR laplacian-filtered right image
 * @param pxX row pixel location
 * @param pxY column pixel location
 * @param state state structure that includes a number of parameters
 *
 * @retval scaled sum of absolute differences for this block -- 
 *      the value is the sum/numberOfPixels
 */
int BarryMoore::GetSAD(Mat leftImage, Mat rightImage, Mat laplacianL, Mat laplacianR, int pxX, int pxY, BarryMooreState state)
{
    // init parameters
    int blockSize = state.blockSize;
    int disparity = state.disparity;
    int sobelLimit = state.sobelLimit;
    
    // top left corner of the SAD box
    int startX = pxX;
    int startY = pxY;
    
    // bottom right corner of the SAD box
    int endX = pxX + blockSize - 1;
    int endY = pxY + blockSize - 1;
    
    #if USE_SAFTEY_CHECKS
        int flag = false;
        if (startX < 0)
        {
            printf("Warning: startX < 0\n");
            flag = true;
        }
        
        if (endX > rightImage.cols)
        {
            printf("Warning: endX > leftImage.cols\n");
            flag = true;
        }
        
        if (startX + disparity < 0)
        {
            printf("Warning: startX + disparity < 0\n");
            flag = true;
        }
        
        if (endX + disparity > rightImage.cols)
        {
            printf("Warning: endX + disparity > leftImage.cols\n");
            flag = true;
        }
        
        if (endX + disparity > rightImage.cols)
        {
            endX = rightImage.cols - disparity;
            flag = true;
        }
        
        if (flag == true)
        {
            printf("startX = %d, endX = %d, disparity = %d\n", startX, endX, disparity);
        }
        
        // disparity might be negative as well
        if (disparity < 0 && startX + disparity < 0)
        {
            startX = -disparity;
        }
        
        startX = max(0, startX);
        startY = max(0, startY);
        
        endX = min(leftImage.cols - disparity, endX);
        endY = min(leftImage.rows, endY);
    #endif
    
    int leftVal = 0, rightVal = 0;
    
    int sad = 0;
    
    for (int i=startY;i<=endY;i++)
    {
        // get a pointer for this row
        /*uchar *this_rowL = leftImage.ptr<uchar>(i);
        uchar *this_rowR = rightImage.ptr<uchar>(i);
        
        uchar *this_row_laplacianL = laplacianL.ptr<uchar>(i);
        uchar *this_row_laplacianR = laplacianR.ptr<uchar>(i);
        */
        for (int j=startX;j<=endX;j++)
        {
            // we are now looking at a single pixel value
            uchar pxL = leftImage.at<uchar>(i,j);
            uchar pxR = rightImage.at<uchar>(i,j + disparity);
            
            uchar sL = laplacianL.at<uchar>(i,j);
            uchar sR = laplacianR.at<uchar>(i,j + disparity);
            
            //uchar pxL = this_rowL[j];
            //uchar pxR = this_rowR[j + disparity];
            
            //uchar sL = this_row_laplacianL[j];//laplacianL.at<uchar>(i,j);
            //uchar sR = this_row_laplacianR[j + disparity]; //laplacianR.at<uchar>(i,j + disparity);
            
            leftVal += sL;
            rightVal += sR;
            
            sad += abs(pxL - pxR);
        }
    }
    int laplacian_value = leftVal + rightVal;
    
    if (leftVal < sobelLimit || rightVal < sobelLimit)
    {
        return -1;
    }
    
    //return sobel;
    return 100*(float)sad/(float)((float)laplacian_value/(float)state.sobelAdd);
}

/**
 * Checks for horizontal invariance by searching near the zero-disparity region
 * for good matches.  If we find a match, that indicates that this is likely not
 * a true obstacle since it matches in more places than just the single-disparity
 * check.
 * 
 * @param leftImage left image
 * @param rightImage right image
 * @param pxX row pixel location
 * @param pxY column pixel location
 * @param state state structure that includes a number of parameters
 *
 * @retval true if there is another match (so NOT an obstacle)
 */
bool BarryMoore::CheckHorizontalInvariance(Mat leftImage, Mat rightImage, Mat sobelL,
    Mat sobelR, int pxX, int pxY, BarryMooreState state) {
    
    // init parameters
    int blockSize = state.blockSize;
    int disparity = state.zero_dist_disparity;
    int sobelLimit = state.sobelLimit;
    
    // top left corner of the SAD box
    int startX = pxX;
    int startY = pxY;
    
    // bottom right corner of the SAD box
    int endX = pxX + blockSize - 1;
    int endY = pxY + blockSize - 1;
    
    // here we check a few spots:
    //  1) the expected match at zero-disparity (10-infinity meters away)
    //  2) inf distance, moved up 1-2 pixels
    //  3) inf distance, moved down 1-2 pixels
    //  4) others?
    
    // first check zero-disparity
    int leftVal = 0;
    
    int right_val_array[400];
    int sad_array[400];
    int sobel_array[400];
    
    for (int i=0;i<400;i++) {
        right_val_array[i] = 0;
        sad_array[i] = 0;
        sobel_array[i] = 0;
    }
    
    int counter = 0;
    
    for (int i=startY;i<=endY;i++)
    {
        for (int j=startX;j<=endX;j++)
        {
            // we are now looking at a single pixel value
            uchar pxL = leftImage.at<uchar>(i,j);
            
            uchar pxR_array[400], sR_array[400];
            
            counter = 0;
            
            for (int vert_offset = INVARIANCE_CHECK_VERT_OFFSET_MIN;
                vert_offset <= INVARIANCE_CHECK_VERT_OFFSET_MAX;
                vert_offset+= INVARIANCE_CHECK_VERT_OFFSET_INCREMENT) {
                
                for (int horz_offset = INVARIANCE_CHECK_HORZ_OFFSET_MIN;
                    horz_offset <= INVARIANCE_CHECK_HORZ_OFFSET_MAX;
                    horz_offset++) {
                    
                    pxR_array[counter] = rightImage.at<uchar>(i + vert_offset, j + disparity + horz_offset);
                    sR_array[counter] = sobelR.at<uchar>(i + vert_offset, j + disparity + horz_offset);
                    right_val_array[counter] += sR_array[counter];
                    
                    sad_array[counter] += abs(pxL - pxR_array[counter]);
                    
                    
                    counter ++;
                }
            }
            
            uchar sL = sobelL.at<uchar>(i,j);
            
            leftVal += sL;
            
        }
    }

    for (int i = 0; i < counter; i++)
    {
        sobel_array[i] = leftVal + right_val_array[i];
        if (right_val_array[i] >= sobelLimit && 100*(float)sad_array[i]/(float)((float)sobel_array[i]/(float)state.sobelAdd) < state.sadThreshold) {
            return true;
        }
    }
    return false;
    
    
}
