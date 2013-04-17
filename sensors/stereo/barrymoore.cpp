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
// values near the edges of images.  Comment for a small speedup.
#define USE_SAFTEY_CHECKS 0


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
        statet[i].leftImage = leftImage; 
        statet[i].rightImage = rightImage;
        
        statet[i].pointVector3d = &pointVector3dArray[i];
        statet[i].pointVector2d = &pointVector2dArray[i];
        statet[i].pointColors = &pointColorsArray[i];
        statet[i].rowOffset = start;
        
         // send in a subset of the map
        statet[i].submapxL = state.mapxL.rowRange(start, end);
        statet[i].submapxR = state.mapxR.rowRange(start, end);
        
        //statet[i].localHitPoints = &(state.localHitPoints[i]);

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
        
        //#if SHOW_DISPLAY
        if (state.show_display)
        {
            pointVector2d->insert( pointVector2d->end(), pointVector2dArray[i].begin(), pointVector2dArray[i].end() );
        }
        //#endif
    }
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
    
    Mat leftImageUnremapped = x->leftImage;
    Mat rightImageUnremapped = x->rightImage;
    cv::vector<Point3f> *pointVector3d = x->pointVector3d;
    cv::vector<Point3i> *pointVector2d = x->pointVector2d;
    cv::vector<uchar> *pointColors = x->pointColors;
    //cv::vector<Point3f> &localHitPoints = *(x->localHitPoints);
    //localHitPoints.clear();
    
    int rowOffset = x->rowOffset;
    
    BarryMooreState state = x->state;
    
    // remap this part of the image
    
    Mat leftImage, rightImage;
    
    remap(leftImageUnremapped, leftImage, x->submapxL, Mat(), INTER_NEAREST);
    remap(rightImageUnremapped, rightImage, x->submapxR, Mat(), INTER_NEAREST);
    
    // apply a sobel filter on everything
    Mat sobelL(leftImage.rows, leftImage.cols, leftImage.depth());
    Mat sobelR(rightImage.rows, rightImage.cols, rightImage.depth());
    
    Sobel(leftImage, sobelL, -1, 1, 1, 3, 1, 0);
    Sobel(rightImage, sobelR, -1, 1, 1, 3, 1, 0);
    
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
    
    for (int i=0; i < leftImage.rows; i+=blockSize)
    {
        for (int j=startJ; j < stopJ; j+=blockSize)
        {
            // get the sum of absolute differences for this location
            // on both images
            int sad = GetSAD(leftImage, rightImage, sobelL, sobelR, j, i, state);
            //int sad = GetHOG(leftImage, rightImage, j, i, state);
            //cout << "x = " << j << " x + disparity = " << j + disparity << endl;
            // check to see if the SAD is below the threshold,
            // indicating a hit
            if (sad < sadThreshold && sad > 10)
            {
                // add it to the vector of matches
                // don't forget to offset it by the blockSize,
                // so we match the center of the block instead
                // of the top left corner
                localHitPoints.push_back(Point3f(j+blockSize/2.0, i+rowOffset+blockSize/2.0, disparity));
                
                uchar pxL = leftImage.at<uchar>(i,j);
                pointColors->push_back(pxL); // TODO: this is the corner of the box, not the center
                
                hitCounter ++;
                
                //#if SHOW_DISPLAY
                if (state.show_display)
                {
                    pointVector2d->push_back(Point3i(j, i+rowOffset, sad));
                }
                //#endif
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

int GetHOG(Mat leftImage, Mat rightImage, int pxX, int pxY, BarryMooreState state)
{
    int disparity = state.disparity;
    HOGDescriptor d, dR;
    vector<float> descriptorsValues, descriptorsValuesR;
    vector<Point> locations, locationsR;
    leftImage = leftImage.colRange(pxX, pxX+64);
    rightImage = rightImage.colRange(pxX+disparity, pxX+64+disparity);
    
    d.compute( leftImage, descriptorsValues, Size(0,0), Size(0,0), locations);
    
    dR.compute( rightImage, descriptorsValuesR, Size(0,0), Size(0,0), locationsR);


    imshow("Depth", get_hogdescriptor_visu(leftImage, descriptorsValues));
    
    imshow("Input2", get_hogdescriptor_visu(rightImage, descriptorsValuesR));

    waitKey();
    
    return 0;

    
    
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
    
    return 100*(float)sad/(float)(sobel + state.sobelAdd);
}

Mat get_hogdescriptor_visu(Mat& origImg, vector<float>& descriptorValues)
{   
    Mat color_origImg;
    cvtColor(origImg, color_origImg, CV_GRAY2RGB);
 
    float zoomFac = 3;
    Mat visu;
    resize(color_origImg, visu, Size(color_origImg.cols*zoomFac, color_origImg.rows*zoomFac));
 
    int blockSize       = 16;
    int cellSize        = 8;
    int gradientBinSize = 9;
    float radRangeForOneBin = M_PI/(float)gradientBinSize; // dividing 180° into 9 bins, how large (in rad) is one bin?
 
    // prepare data structure: 9 orientation / gradient strenghts for each cell
    int cells_in_x_dir = 64 / cellSize;
    int cells_in_y_dir = 128 / cellSize;
    int totalnrofcells = cells_in_x_dir * cells_in_y_dir;
    float*** gradientStrengths = new float**[cells_in_y_dir];
    int** cellUpdateCounter   = new int*[cells_in_y_dir];
    for (int y=0; y<cells_in_y_dir; y++)
    {
        gradientStrengths[y] = new float*[cells_in_x_dir];
        cellUpdateCounter[y] = new int[cells_in_x_dir];
        for (int x=0; x<cells_in_x_dir; x++)
        {
            gradientStrengths[y][x] = new float[gradientBinSize];
            cellUpdateCounter[y][x] = 0;
 
            for (int bin=0; bin<gradientBinSize; bin++)
                gradientStrengths[y][x][bin] = 0.0;
        }
    }
 
    // nr of blocks = nr of cells - 1
    // since there is a new block on each cell (overlapping blocks!) but the last one
    int blocks_in_x_dir = cells_in_x_dir - 1;
    int blocks_in_y_dir = cells_in_y_dir - 1;
 
    // compute gradient strengths per cell
    int descriptorDataIdx = 0;
    int cellx = 0;
    int celly = 0;
 
    for (int blockx=0; blockx<blocks_in_x_dir; blockx++)
    {
        for (int blocky=0; blocky<blocks_in_y_dir; blocky++)            
        {
            // 4 cells per block ...
            for (int cellNr=0; cellNr<4; cellNr++)
            {
                // compute corresponding cell nr
                int cellx = blockx;
                int celly = blocky;
                if (cellNr==1) celly++;
                if (cellNr==2) cellx++;
                if (cellNr==3)
                {
                    cellx++;
                    celly++;
                }
 
                for (int bin=0; bin<gradientBinSize; bin++)
                {
                    float gradientStrength = descriptorValues[ descriptorDataIdx ];
                    descriptorDataIdx++;
 
                    gradientStrengths[celly][cellx][bin] += gradientStrength;
 
                } // for (all bins)
 
 
                // note: overlapping blocks lead to multiple updates of this sum!
                // we therefore keep track how often a cell was updated,
                // to compute average gradient strengths
                cellUpdateCounter[celly][cellx]++;
 
            } // for (all cells)
 
 
        } // for (all block x pos)
    } // for (all block y pos)
 
 
    // compute average gradient strengths
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
 
            float NrUpdatesForThisCell = (float)cellUpdateCounter[celly][cellx];
 
            // compute average gradient strenghts for each gradient bin direction
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                gradientStrengths[celly][cellx][bin] /= NrUpdatesForThisCell;
            }
        }
    }
 
 
    cout << "descriptorDataIdx = " << descriptorDataIdx << endl;
 
    // draw cells
    for (int celly=0; celly<cells_in_y_dir; celly++)
    {
        for (int cellx=0; cellx<cells_in_x_dir; cellx++)
        {
            int drawX = cellx * cellSize;
            int drawY = celly * cellSize;
 
            int mx = drawX + cellSize/2;
            int my = drawY + cellSize/2;
 
            rectangle(visu, Point(drawX*zoomFac,drawY*zoomFac), Point((drawX+cellSize)*zoomFac,(drawY+cellSize)*zoomFac), CV_RGB(100,100,100), 1);
 
            // draw in each cell all 9 gradient strengths
            for (int bin=0; bin<gradientBinSize; bin++)
            {
                float currentGradStrength = gradientStrengths[celly][cellx][bin];
 
                // no line to draw?
                if (currentGradStrength==0)
                    continue;
 
                float currRad = bin * radRangeForOneBin + radRangeForOneBin/2;
 
                float dirVecX = cos( currRad );
                float dirVecY = sin( currRad );
                float maxVecLen = cellSize/2;
                float scale = 2.5; // just a visualization scale, to see the lines better
 
                // compute line coordinates
                float x1 = mx - dirVecX * currentGradStrength * maxVecLen * scale;
                float y1 = my - dirVecY * currentGradStrength * maxVecLen * scale;
                float x2 = mx + dirVecX * currentGradStrength * maxVecLen * scale;
                float y2 = my + dirVecY * currentGradStrength * maxVecLen * scale;
 
                // draw gradient visualization
                line(visu, Point(x1*zoomFac,y1*zoomFac), Point(x2*zoomFac,y2*zoomFac), CV_RGB(0,255,0), 1);
 
            } // for (all bins)
 
        } // for (cellx)
    } // for (celly)
 
 
    // don't forget to free memory allocated by helper data structures!
    for (int y=0; y<cells_in_y_dir; y++)
    {
      for (int x=0; x<cells_in_x_dir; x++)
      {
           delete[] gradientStrengths[y][x];            
      }
      delete[] gradientStrengths[y];
      delete[] cellUpdateCounter[y];
    }
    delete[] gradientStrengths;
    delete[] cellUpdateCounter;
 
    return visu;
 
} // get_hogdescriptor_visu
