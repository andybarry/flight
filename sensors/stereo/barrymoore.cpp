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
#define USE_SAFTEY_CHECKS 1

#define NUM_THREADS 8
#define NUM_REMAP_THREADS 8

#define INVARIANCE_CHECK_VERT_OFFSET_MIN (-8)
#define INVARIANCE_CHECK_VERT_OFFSET_MAX 8
#define INVARIANCE_CHECK_VERT_OFFSET_INCREMENT 2

#define INVARIANCE_CHECK_HORZ_OFFSET_MIN (-2)
#define INVARIANCE_CHECK_HORZ_OFFSET_MAX 2


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
void StereoBarryMoore(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<uchar> *pointColors, cv::vector<Point3i> *pointVector2d, BarryMooreState state)
{
    Mat leftImage = _leftImage.getMat();
    Mat rightImage = _rightImage.getMat();
    
    // make sure that the inputs are of the right type.
    CV_Assert(leftImage.type() == CV_8UC1 && rightImage.type() == CV_8UC1);
    
    // we want to use the sum-of-absolute-differences (SAD) algorithm
    // on a single disparity
    
    // split things up so we can parallelize
    int rows = leftImage.rows;
    
    // first parallelize remaping
    pthread_t remap_threads[NUM_REMAP_THREADS+1];
    RemapThreadState remap_states[NUM_REMAP_THREADS+1];
    
    // we split these arrays up and send them into each
    // thread so at the end, each thread has written to the
    // appropriate spot in the array
    Mat remapped_left(state.mapxL.rows, state.mapxL.cols, leftImage.depth());
    Mat remapped_right(state.mapxR.rows, state.mapxR.cols, rightImage.depth());
    
    Mat laplacian_left(state.mapxL.rows, state.mapxL.cols, leftImage.depth());
    Mat laplacian_right(state.mapxR.rows, state.mapxR.cols, rightImage.depth());
    
    for (int i = 0; i< NUM_REMAP_THREADS; i++) {
        
        int start = rows/NUM_REMAP_THREADS*i;
        int end = rows/NUM_REMAP_THREADS*(i+1);
    
        remap_states[i].leftImage = leftImage;
        remap_states[i].rightImage = rightImage;
        
        // send in a subset of the map
        remap_states[i].submapxL = state.mapxL.rowRange(start, end);
        remap_states[i].submapxR = state.mapxR.rowRange(start, end);
        
        // send in subsets of the arrays to be filled in
        remap_states[i].sub_remapped_left_image = remapped_left.rowRange(start, end);
        
        remap_states[i].sub_remapped_right_image = remapped_right.rowRange(start, end);
        
        remap_states[i].sub_laplacian_left = laplacian_left.rowRange(start, end);
        remap_states[i].sub_laplacian_right = laplacian_right.rowRange(start, end);
        
        // fire the remap thread
        pthread_create(&remap_threads[i], NULL, RemapThreaded, &remap_states[i]);
    }
    
    
    // wait for all remapping threads to finish
    // and join the remapped pieces back together
    for (int i=0;i<NUM_REMAP_THREADS;i++)
    {
        pthread_join(remap_threads[i], NULL);

        /*
        remapped_left.push_back(remap_states[i].sub_remaped_left_image);
        remapped_right.push_back(remap_states[i].sub_remaped_right_image);
        
        laplacian_left.push_back(remap_states[i].sub_laplacian_left);
        laplacian_right.push_back(remap_states[i].sub_laplacian_right);
        */
    }
    
    // now we have fully remapped both images
    //imshow("Left Block", laplacian_left);
    //imshow("Right Block", laplacian_right);
    
    
    
    
    pthread_t thread[NUM_THREADS+1];
    
    BarryMooreStateThreaded statet[NUM_THREADS+1];
    
    cv::vector<Point3f> pointVector3dArray[NUM_THREADS+1];
    cv::vector<Point3i> pointVector2dArray[NUM_THREADS+1];
    cv::vector<uchar> pointColorsArray[NUM_THREADS+1];
    
    for (int i=0;i<NUM_THREADS;i++)
    {
        
        statet[i].state = state;
        
        int start = rows/NUM_THREADS*i;
        int end = rows/NUM_THREADS*(i+1);
        
        // send in the whole image because each thread needs
        // the entire image to do its small remapping job
        statet[i].remapped_left = remapped_left; 
        statet[i].remapped_right = remapped_right;
        
        statet[i].laplacian_left = laplacian_left;
        statet[i].laplacian_right = laplacian_right;
        
        statet[i].pointVector3d = &pointVector3dArray[i];
        statet[i].pointVector2d = &pointVector2dArray[i];
        statet[i].pointColors = &pointColorsArray[i];
        statet[i].row_start = start;
        statet[i].row_end = end;
        
        // fire the thread      
        pthread_create( &thread[i], NULL, StereoBarryMooreThreaded, &statet[i]);
    }
    
    // wait for all the threads to come back
    for (int i=0;i<NUM_THREADS;i++)
    {
        pthread_join( thread[i], NULL);
    }
    
    
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

/**
 * Thread that remaps images
 *
 */
void* RemapThreaded(void *x) {
    // do remapping
    
    RemapThreadState *remap_state = (RemapThreadState*) x;
    
    // remap this part of the image
    
    remap(remap_state->leftImage, remap_state->sub_remapped_left_image, remap_state->submapxL, Mat(), INTER_NEAREST);
    remap(remap_state->rightImage, remap_state->sub_remapped_right_image, remap_state->submapxR, Mat(), INTER_NEAREST);
    
    // apply interest filtering to this part of the image
    // apply a sobel filter on everything
    
    Laplacian(remap_state->sub_remapped_left_image, remap_state->sub_laplacian_left, -1, 3);
    Laplacian(remap_state->sub_remapped_right_image, remap_state->sub_laplacian_right, -1, 3);
    
    
    // exit the thread
    return NULL;
}

/**
 * Thread that actually does the work for the BarryMoore algorithm.
 *
 * @param statet all the parameters are set
 *          as a BarryMooreStateThreaded struct.
 *
 * @retval will always be NULL since the real values are passed
 *      back in the vector that is in statet.
 */
void* StereoBarryMooreThreaded(void *statet)
{

    BarryMooreStateThreaded *x = (BarryMooreStateThreaded*) statet;
    
    Mat leftImage = x->remapped_left;
    Mat rightImage = x->remapped_right;
    Mat laplacian_left = x->laplacian_left;
    Mat laplacian_right = x->laplacian_right;
    
    cv::vector<Point3f> *pointVector3d = x->pointVector3d;
    cv::vector<Point3i> *pointVector2d = x->pointVector2d;
    cv::vector<uchar> *pointColors = x->pointColors;
    
    int row_start = x->row_start;
    int row_end = x->row_end;
    
    BarryMooreState state = x->state;
    
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
    
    return NULL; // exit the thread
}


/**
 * Get the sum of absolute differences for a specific pixel location and disparity
 *
 * @param leftImage left image
 * @param rightImage right image
 * @param pxX row pixel location
 * @param pxY column pixel location
 * @param state state structure that includes a number of parameters
 *
 * @retval scaled sum of absolute differences for this block -- 
 *      the value is the sum/numberOfPixels
 */
int GetSAD(Mat leftImage, Mat rightImage, Mat sobelL, Mat sobelR, int pxX, int pxY, BarryMooreState state)
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
        for (int j=startX;j<=endX;j++)
        {
            // we are now looking at a single pixel value
            uchar pxL = leftImage.at<uchar>(i,j);
            uchar pxR = rightImage.at<uchar>(i,j + disparity);
            
            uchar sL = sobelL.at<uchar>(i,j);
            uchar sR = sobelR.at<uchar>(i,j + disparity);
            
            leftVal += sL;
            rightVal += sR;
            
            sad += abs(pxL - pxR);
        }
    }
    int sobel = leftVal + rightVal;
    
    if (leftVal < sobelLimit || rightVal < sobelLimit)
    {
        return -1;
    }
    
    //return sobel;
    return 100*(float)sad/(float)((float)sobel/(float)state.sobelAdd);
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
bool CheckHorizontalInvariance(Mat leftImage, Mat rightImage, Mat sobelL,
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
