/**
 * Implements pushbroom stereo, a fast, single-disparity stereo algorithm.
 *
 * Copyright 2013-2015, Andrew Barry <abarry@csail.mit.edu>
 *
 */

#include "pushbroom-stereo.hpp"
#include <pthread.h>

// if USE_SAFTEY_CHECKS is 1, GetSAD will try to make sure
// that it will do the right thing even if you ask it for pixel
// values near the edges of images.  Set to 0 for a small speedup.
#define USE_SAFTEY_CHECKS 0

#define INVARIANCE_CHECK_VERT_OFFSET_MIN (-8)
#define INVARIANCE_CHECK_VERT_OFFSET_MAX 8
#define INVARIANCE_CHECK_VERT_OFFSET_INCREMENT 2

#define INVARIANCE_CHECK_HORZ_OFFSET_MIN (-3)
#define INVARIANCE_CHECK_HORZ_OFFSET_MAX 3

#define NUMERIC_CONST 333 // just a constant that we multiply the score by to make
                          // all the parameters in a nice integer range

PushbroomStereoThreadStarter thread_starter[NUM_THREADS+1];


PushbroomStereo::PushbroomStereo() {
    // init worker threads



    for (int i = 0; i < NUM_THREADS; i++) {
        // start all the worker threads


        data_mutexes_[i].lock();
        SetHasNewData(i, false);

        thread_starter[i].thread_number = i;
        thread_starter[i].data_mutex = &(data_mutexes_[i]);
        thread_starter[i].cv_new_data = &(cv_new_data_[i]);
        thread_starter[i].cv_thread_finish = &(cv_thread_finish_[i]);
        thread_starter[i].parent = this;

        lockers_[i] = unique_lock<mutex>(data_mutexes_[i], std::defer_lock);

        // start the thread
        pthread_create(&(worker_pool_[i]), NULL, WorkerThread, &(thread_starter[i]));

    }
}

void* PushbroomStereo::WorkerThread(void *x) {

    PushbroomStereoThreadStarter *statet = (PushbroomStereoThreadStarter*) x;

    mutex *data_mutex = statet->data_mutex;
    int thread_number = statet->thread_number;
    condition_variable *cv_new_data = statet->cv_new_data;
    condition_variable *cv_thread_finish = statet->cv_thread_finish;

    PushbroomStereo *parent = statet->parent;

    unique_lock<mutex> locker(*data_mutex, std::defer_lock);

    while (true) {

        // wait on the condition variable for
        // "has new data"
        data_mutex->lock();
        //cout << "[thread locked]" << endl;

        //cout << "[thread " << thread_number << "] waiting" << endl;
        while (parent->GetHasNewData(thread_number) == false) {
            //cout << "[thread " << thread_number << "] inside while loop" << endl;
            //cout << "[thread " << thread_number << "] ready for notifictaions" << endl;
            //cout << "[thread unlocked]" << endl;
            cv_new_data->wait(locker);
            //cout << "[thread locked]" << endl;
            //cout << "[thread " << thread_number << "] after wait in while loop" << endl;
        }

        // if we don't already have the mutex, get it

        //cout << "[thread " << thread_number << "] going" << endl;


        // if we're here, there's work to be done
        //cout << "[thread " << thread_number << "] running" << endl;

        // see if that work is remapping or stereo processing
        switch (parent->GetWorkType(thread_number)) {
            case REMAP:
                parent->RunRemapping(parent->GetRemapState(thread_number));
                break;

            case INTEREST_OP:
                parent->RunInterestOp(parent->GetInterestOpState(thread_number));
                break;

            case STEREO:
                parent->RunStereoPushbroomStereo(parent->GetThreadedState(thread_number));
                break;

            default:
                cerr << "Warning: unknown thread work type." << endl;
                break;
        }

        // done, signal the waiting main thread

        parent->SetHasNewData(thread_number, false);
        cv_thread_finish->notify_one();

        data_mutex->unlock();
        //cout << "[thread unlocked]" << endl;

        //cout << "[thread " << thread_number << "] done" << endl;

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
void PushbroomStereo::ProcessImages(InputArray _leftImage, InputArray _rightImage, cv::vector<Point3f> *pointVector3d, cv::vector<uchar> *pointColors, cv::vector<Point3i> *pointVector2d, PushbroomStereoState state) {

    //cout << "[main] entering process images" << endl;

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
    //cout << "[main] all remap threads started" << endl;


    // wait for all remapping threads to finish
    SyncWorkerThreads();

    //cout << "[main] all remap threads finished" << endl;


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
    //imshow("Left Block", remapped_right);
    //imshow("Right Block", laplacian_right);

    //cout << "[main] imshow2 ok" << endl;

    cv::vector<Point3f> pointVector3dArray[NUM_THREADS+1];
    cv::vector<Point3i> pointVector2dArray[NUM_THREADS+1];
    cv::vector<uchar> pointColorsArray[NUM_THREADS+1];

    //cout << "[main] firing worker threads..." << endl;

    if (state.lastValidPixelRow > 0) {

        // crop image to be only include valid pixels
        rows = state.lastValidPixelRow;
    }


    // figure out how to split up the work
    int thread_increment = RoundUp(rows/NUM_THREADS, state.blockSize);
    int last_thread_num_rows = rows - thread_increment * (NUM_THREADS - 1);

    // make sure the last thread has a number of rows divisible by the block size
    last_thread_num_rows = last_thread_num_rows - last_thread_num_rows % state.blockSize;

    //printf("thread increment: %d, last thread: %d\n", thread_increment, last_thread_num_rows);

    for (int i=0;i<NUM_THREADS;i++)
    {

        thread_states_[i].state = state;

        int start = thread_increment * i;
        int end;

        if (i < NUM_THREADS - 1) {
            // not the last thread
            end = thread_increment*(i+1) - 1;
        } else {
            // the last thread
            end = start + last_thread_num_rows - 1;
        }

        //printf("start: %d, end: %d\n", start, end);

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

    //cout << "[main] got all stereo" << endl;

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

void PushbroomStereo::StartWorkerThread(int i, ThreadWorkType work_type) {

    work_type_[i] = work_type;

    SetHasNewData(i, true);

    data_mutexes_[i].unlock();
    //cout << "[main unlocked]" << endl;


    //cout << "[main] notifying [" << i << "]" << endl;
    cv_new_data_[i].notify_one();
}

void PushbroomStereo::SyncWorkerThreads() {

    //cout << "[main] in sync workers" << endl;
    for (int i=0;i<NUM_THREADS;i++) {

        //cout << "[main] trying for lock [" << i << "]" << endl;
        data_mutexes_[i].lock();
        //cout << "[main locked]" << endl;

        while (GetHasNewData(i) == true) {
            //cout << "[main] waiting on [" << i << "]" << endl;
            // wait for the thread to finish
            //cout << "[main unlocked]" << endl;
            cv_thread_finish_[i].wait(lockers_[i]);
            //cout << "[main locked]" << endl;
        }
    }
    //cout << "[main] leaving sync workers" << endl;
}

/**
 * Function (for running in a thread) that remaps images
 *
 */
void PushbroomStereo::RunRemapping(RemapThreadState *remap_state) {

    // remap this part of the image

    remap(remap_state->left_image, remap_state->sub_remapped_left_image, remap_state->submapxL, Mat(), INTER_NEAREST);
    remap(remap_state->right_image, remap_state->sub_remapped_right_image, remap_state->submapxR, Mat(), INTER_NEAREST);



}

void PushbroomStereo::RunInterestOp(InterestOpState *interest_state) {

    // apply interest operator
    Laplacian(interest_state->left_image.rowRange(interest_state->row_start, interest_state->row_end), interest_state->sub_laplacian_left, -1, 3, 1, 0, BORDER_DEFAULT);

    Laplacian(interest_state->right_image.rowRange(interest_state->row_start, interest_state->row_end), interest_state->sub_laplacian_right, -1, 3, 1, 0, BORDER_DEFAULT);

}

/**
 * Function that actually does the work for the PushbroomStereo algorithm.
 *
 * @param statet all the parameters are set
 *          as a PushbroomStereoStateThreaded struct.
 *
 * @retval will always be NULL since the real values are passed
 *      back in the vector that is in statet.
 */
void PushbroomStereo::RunStereoPushbroomStereo(PushbroomStereoStateThreaded *statet)
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

    PushbroomStereoState state = statet->state;

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

    //printf("row_start: %d, row_end: %d, startJ: %d, stopJ: %d, rows: %d, cols: %d\n", row_start, row_end, startJ, stopJ, leftImage.rows, leftImage.cols);

    int hitCounter = 0;


    if (state.random_results < 0) {
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

                    if (!state.check_horizontal_invariance || CheckHorizontalInvariance(leftImage, rightImage, laplacian_left, laplacian_right, j, i, state) == false) {

                        // add it to the vector of matches
                        // don't forget to offset it by the blockSize,
                        // so we match the center of the block instead
                        // of the top left corner
                        localHitPoints.push_back(Point3f(j+blockSize/2.0, i+blockSize/2.0, -disparity));

                        //localHitPoints.push_back(Point3f(state.debugJ, state.debugI, -disparity));


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
    } else {

        double intpart;

        float fractpart = modf(state.random_results , &intpart);
        hitCounter = round(intpart);

        // determine if this is a time we'll use that last point
        std::random_device rd;
        std::default_random_engine generator(rd()); // rd() provides a random seed
        std::uniform_real_distribution<float> distribution(0, 1);

        if (fractpart > distribution(generator)) {
            hitCounter ++;
        }

        for (int i = 0; i < hitCounter; i++) {

            int randx = rand() % (stopJ - startJ) + startJ;
            int randy = rand() % (row_end - row_start) + row_start;

            localHitPoints.push_back(Point3f(randx, randy, -disparity));
        }
    }

    // now we have an array of hits -- transform them to 3d points
    if (hitCounter > 0) {

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
 * @param left_interest optional parameter that will be filled with the value for the left interest operation
 * @param right_interest same as above, for the right image
 *
 * @retval scaled sum of absolute differences for this block --
 *      the value is the sum/numberOfPixels
 */
int PushbroomStereo::GetSAD(Mat leftImage, Mat rightImage, Mat laplacianL, Mat laplacianR, int pxX, int pxY, PushbroomStereoState state, int *left_interest, int *right_interest, int *raw_sad)
{
    // init parameters
    int blockSize = state.blockSize;
    int disparity = state.disparity;
    int sobelLimit = state.sobelLimit;

    // top left corner of the SAD box
    int startX = pxX;
    int startY = pxY;

    // bottom right corner of the SAD box
    #ifndef USE_NEON
        int endX = pxX + blockSize - 1;
    #endif

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
            printf("Warning: endX + disparity > rightImage.cols\n");
            endX = rightImage.cols - disparity;
            flag = true;
        }

        if (startY < 0) {
            printf("Warning: startY < 0\n");
            flag = true;
        }

        if (endY > rightImage.rows) {
            printf("Warning: endY > rightImage.rows\n");
            flag = true;
        }

        // disparity might be negative as well
        if (disparity < 0 && startX + disparity < 0)
        {
            printf("Warning: disparity < 0 && startX + disparity < 0\n");
            startX = -disparity;
            flag = true;
        }

        if (flag == true)
        {
            printf("startX = %d, endX = %d, disparity = %d, startY = %d, endY = %d\n", startX, endX, disparity, startY, endY);
        }



        startX = max(0, startX);
        startY = max(0, startY);

        endX = min(leftImage.cols - disparity, endX);
        endY = min(leftImage.rows, endY);
    #endif

    //printf("startX = %d, endX = %d, disparity = %d, startY = %d, endY = %d, rows = %d, cols = %d\n", startX, endX, disparity, startY, endY, leftImage.rows, leftImage.cols);

    int leftVal = 0, rightVal = 0;

    int sad = 0;

    #ifdef USE_NEON
        uint16x8_t interest_op_sum_8x_L, interest_op_sum_8x_R, sad_sum_8x;

        // load zeros into everything
        interest_op_sum_8x_L = vdupq_n_u16(0);
        interest_op_sum_8x_R = vdupq_n_u16(0);
        sad_sum_8x = vdupq_n_u16(0);

    #endif

    for (int i=startY;i<=endY;i++) {
        // get a pointer for this row
        uchar *this_rowL = leftImage.ptr<uchar>(i);
        uchar *this_rowR = rightImage.ptr<uchar>(i);

        uchar *this_row_laplacianL = laplacianL.ptr<uchar>(i);
        uchar *this_row_laplacianR = laplacianR.ptr<uchar>(i);

        #ifdef USE_NEON
            // load this row into memory
            uint8x8_t this_row_8x8_L = vld1_u8(this_rowL + startX);
            uint8x8_t this_row_8x8_R = vld1_u8(this_rowR + startX + disparity);

            uint8x8_t interest_op_8x8_L = vld1_u8(this_row_laplacianL + startX);
            uint8x8_t interest_op_8x8_R = vld1_u8(this_row_laplacianR + startX + disparity);

            // do absolute differencing for the entire row in one operation!
            uint8x8_t sad_8x = vabd_u8(this_row_8x8_L, this_row_8x8_R);

            // sum up
            sad_sum_8x = vaddw_u8(sad_sum_8x, sad_8x);

            // sum laplacian values
            interest_op_sum_8x_L = vaddw_u8(interest_op_sum_8x_L, interest_op_8x8_L);
            interest_op_sum_8x_R = vaddw_u8(interest_op_sum_8x_R, interest_op_8x8_R);

        #else // USE_NEON

            for (int j=startX;j<=endX;j++) {
                // we are now looking at a single pixel value
                /*uchar pxL = leftImage.at<uchar>(i,j);
                uchar pxR = rightImage.at<uchar>(i,j + disparity);

                uchar sL = laplacianL.at<uchar>(i,j);
                uchar sR = laplacianR.at<uchar>(i,j + disparity);
                */


                uchar sL = this_row_laplacianL[j];//laplacianL.at<uchar>(i,j);
                uchar sR = this_row_laplacianR[j + disparity]; //laplacianR.at<uchar>(i,j + disparity);

                leftVal += sL;
                rightVal += sR;

                uchar pxL = this_rowL[j];
                uchar pxR = this_rowR[j + disparity];

                sad += abs(pxL - pxR);
            }
        #endif // USE_NEON
    }

    #ifdef USE_NEON
        // sum up
        sad = vgetq_lane_u16(sad_sum_8x, 0) + vgetq_lane_u16(sad_sum_8x, 1)
           + vgetq_lane_u16(sad_sum_8x, 2) + vgetq_lane_u16(sad_sum_8x, 3)
           + vgetq_lane_u16(sad_sum_8x, 4);// + vgetq_lane_u16(sad_sum_8x, 5)
    //           + vgetq_lane_u16(sad_sum_8x, 6) + vgetq_lane_u16(sad_sum_8x, 7);

        leftVal = vgetq_lane_u16(interest_op_sum_8x_L, 0)
                + vgetq_lane_u16(interest_op_sum_8x_L, 1)
                + vgetq_lane_u16(interest_op_sum_8x_L, 2)
                + vgetq_lane_u16(interest_op_sum_8x_L, 3)
                + vgetq_lane_u16(interest_op_sum_8x_L, 4);


        rightVal = vgetq_lane_u16(interest_op_sum_8x_R, 0)
                 + vgetq_lane_u16(interest_op_sum_8x_R, 1)
                 + vgetq_lane_u16(interest_op_sum_8x_R, 2)
                 + vgetq_lane_u16(interest_op_sum_8x_R, 3)
                 + vgetq_lane_u16(interest_op_sum_8x_R, 4);
    #endif

    //cout << "(" << leftVal << ", " << rightVal << ") vs. (" << leftVal2 << ", " << rightVal2 << ")" << endl;

    int laplacian_value = leftVal + rightVal;

    //cout << "sad with neon: " << sad << " without neon: " << sad2 << endl;

    if (left_interest != NULL) {
        *left_interest = leftVal;
    }

    if (right_interest != NULL) {
        *right_interest = rightVal;
    }

    // percentage of total interest value that is different
    //float diff_score = 100*(float)abs(leftVal - rightVal)/(float)laplacian_value;

    if (raw_sad != NULL) {
        *raw_sad = sad;
    }


    if (leftVal < sobelLimit || rightVal < sobelLimit)// || diff_score > state.interest_diff_limit)
    {
        return -1;
    }

    // weight laplacian_value into the score

    //return sobel;
    return NUMERIC_CONST*(float)sad/(float)laplacian_value;
}

/**
 * Checks for horizontal invariance by searching near the zero-disparity region
 * for good matches.  If we find a match, that indicates that this is likely not
 * a true obstacle since it matches in more places than just the single-disparity
 * check.
 *
 * @param leftImage left image
 * @param rightImage right image
 * @param pxX column pixel location
 * @param pxY row pixel location
 * @param state state structure that includes a number of parameters
 *
 * @retval true if there is another match (so NOT an obstacle)
 */
bool PushbroomStereo::CheckHorizontalInvariance(Mat leftImage, Mat rightImage, Mat sobelL,
    Mat sobelR, int pxX, int pxY, PushbroomStereoState state) {

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


    // if we are off the edge of the image so we can't tell if this
    // might be an issue -- give up and return true
    // (note: this used to be false and caused bad detections on real flight
    // data near the edge of the frame)
    if (   startX + disparity + INVARIANCE_CHECK_HORZ_OFFSET_MIN < 0
        || endX + disparity + INVARIANCE_CHECK_HORZ_OFFSET_MAX > rightImage.cols) {

        return true;
    }

    if (startY + INVARIANCE_CHECK_VERT_OFFSET_MIN < 0
        || endY + INVARIANCE_CHECK_VERT_OFFSET_MAX > rightImage.rows) {
        // we are limited in the vertical range we can check here

        // TODO: be smarter here

        // give up and bail out, deleting potential hits
        return true;

    }


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

            // for each pixel in the left image, we are going to search a bunch
            // of pixels in the right image.  We do it this way to save the computation
            // of dealing with the same left-image pixel over and over again.

            // counter indexes which location we're looking at for this run, so for each
            // pixel in the left image, we examine a bunch of pixels in the right image
            // and add up their results into different slots in sad_array over the loop
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

        // we don't check for leftVal >= sobelLimit because we have already
        // checked that in the main search loop (in GetSAD).
        //if (right_val_array[i] >= sobelLimit && 100*(float)sad_array[i]/(float)((float)sobel_array[i]*state.interestOperatorMultiplierHorizontalInvariance) < state.sadThreshold) {
        if (right_val_array[i] >= sobelLimit && NUMERIC_CONST*state.horizontalInvarianceMultiplier*(float)sad_array[i]/((float)sobel_array[i]) < state.sadThreshold) {
            return true;
        }
    }
    return false;


}

/**
 * Round up to the nearest multiple of a number.
 * From: http://stackoverflow.com/questions/3407012/c-rounding-up-to-the-nearest-multiple-of-a-number
 *
 * @param numToRound input number to be rounded
 * @param multiple multiple of the number to be rounded to
 *
 * @retval rounded number
 *
 * Examples:
 *  roundUp(7, 100) --> 100
 *  roundUp(52, 20) --> 60
 *
 */
int PushbroomStereo::RoundUp(int numToRound, int multiple)
{
    if (multiple == 0)
        return numToRound;

    int remainder = abs(numToRound) % multiple;
    if (remainder == 0)
        return numToRound;
    if (numToRound < 0)
        return -(abs(numToRound) - remainder);
    return numToRound + multiple - remainder;
}
