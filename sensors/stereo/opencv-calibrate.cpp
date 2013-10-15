/*
 * Use Gtk+ to show live video from the dc1394 camera
 *
 * Written by John Stowers <john.stowers@gmail.com>
 * Modified for STEREO VISION PROCESSING BY ANDREW BARRY <abarry@csail.mit.edu>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 
#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <sys/time.h>

#include "opencv2/opencv.hpp"

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


int camera_custom_setup(dc1394camera_t *camera);

int main(int argc, char *argv[])
{
    dc1394_t        *d;
    dc1394camera_t  *camera;
    IplImage        *frame;
    dc1394error_t   err;
    guint64         guid = 0x00b09d0100a01a9a; //0x00b09d0100af04d8;
    
    IplImage *frameArray[MAX_FRAMES];
    IplImage *frameArray2[MAX_FRAMES];
    
    dc1394video_frame_t *frame_raw1;
    dc1394video_frame_t *frame_raw2;
    
    unsigned long elapsed;
    int numFrames = 0;
    
    // ----- cam 2 -----
    dc1394_t        *d2;
    dc1394camera_t  *camera2;
    IplImage        *frame2;
    dc1394error_t   err2;
    guint64         guid2 = 0x00b09d0100a01ac5; //0x00b09d0100a01ac5;

    d = dc1394_new ();
    if (!d)
        g_critical("Could not create dc1394 context");
        
    d2 = dc1394_new ();
    if (!d2)
        g_critical("Could not create dc1394 context for camera 2");

    camera = dc1394_camera_new (d, guid);
    if (!camera)
        g_critical("Could not create dc1394 camera");

    camera2 = dc1394_camera_new (d2, guid2);
    if (!camera2)
        g_critical("Could not create dc1394 camera for camera 2");

    // setup
    err = setup_gray_capture(camera, DC1394_VIDEO_MODE_FORMAT7_1);
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not setup camera");

    err2 = setup_gray_capture(camera2, DC1394_VIDEO_MODE_FORMAT7_1);
    DC1394_ERR_CLN_RTN(err2, cleanup_and_exit(camera2), "Could not setup camera number 2");
    
    // enable auto-exposure
    // turn on the auto exposure feature
    err = dc1394_feature_set_power(camera, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not turn on the exposure feature");
    
    err = dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    DC1394_ERR_RTN(err,"Could not turn on Auto-exposure");
    
    // enable auto-exposure
    // turn on the auto exposure feature
    err = dc1394_feature_set_power(camera2, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not turn on the exposure feature for cam2");
    
    err = dc1394_feature_set_mode(camera2, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
    DC1394_ERR_RTN(err,"Could not turn on Auto-exposure for cam2");
    
    // enable camera
    err = dc1394_video_set_transmission(camera, DC1394_ON);
    DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not start camera iso transmission");
    err2 = dc1394_video_set_transmission(camera2, DC1394_ON);
    DC1394_ERR_CLN_RTN(err2, cleanup_and_exit(camera2), "Could not start camera iso transmission for camera number 2");

	cvNamedWindow("Input", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Input2", CV_WINDOW_AUTOSIZE);
	cvMoveWindow("Input2", 500, 100);
	
    CvVideoWriter *writer = NULL;
    CvVideoWriter *writer2 = NULL;
    
    CvSize size;
    
    Mat corners, cornersR;
    
    int i;
    while (numFrames < MAX_FRAMES) {
        // each loop dump a bunch of frames to clear the buffer
        for (i=0;i<10;i++)
        {
            frame = dc1394_capture_get_iplimage(camera);
            frame2 = dc1394_capture_get_iplimage(camera2);
        }
        
        IplImage *image_left = frame;
        IplImage *image_right = frame2;
        // image_left and image_right are the input 8-bit
        // single-channel images
        // from the left and the right cameras, respectively
        
         size = cvGetSize(image_left);
        
        
        // copy the images for drawing/display
        Mat chessL = Mat(image_left, true);
        Mat chessR = Mat(image_right, true);
        Mat chessLc;
        chessLc.create(size, CV_32FC3);
        Mat chessRc;
        chessRc.create(size, CV_32FC3);

        // attempt checkerboard matching
        bool foundPattern = findChessboardCorners((Mat)image_left, Size(CHESS_X, CHESS_Y), corners);
        
        foundPattern = foundPattern & findChessboardCorners((Mat)image_right, Size(CHESS_X, CHESS_Y), cornersR);
        
        
        cvtColor( chessL, chessLc, CV_GRAY2BGR );
        cvtColor( chessR, chessRc, CV_GRAY2BGR );
        drawChessboardCorners(chessLc, Size(CHESS_X, CHESS_Y), corners, foundPattern);
        
        drawChessboardCorners(chessRc, Size(CHESS_X, CHESS_Y), cornersR, foundPattern);

        imshow("Input", chessLc);
	    imshow("Input2", chessRc);
		
		// key codes:
		// page up: 654365
		// page down: 65366
		// b: 98
		
		char key = waitKey();
		//printf("%d\n", (int)key);
		if (key == 98)
		{
		    break;
		} else if (key == 86){
		    // this was a good one -- save it
		    frameArray[numFrames] = image_left;
            frameArray2[numFrames] = image_right;

            printf("Saved frame %d\n", numFrames);
            
            numFrames ++;
		}
		
	}

    printf("\n\n");
    
    //writer = cvCreateVideoWriter("testvid-uc.avi", CV_FOURCC('I','4','2','0'), 30, size, 0 );
    //writer2 = cvCreateVideoWriter("testvid-uc2.avi", CV_FOURCC('I','4','2','0'), 30, size, 0 );
    
    char filename[1000];
    
    for (i=0;i<numFrames;i++)
    {
        printf("hi\n");
        //cvWriteFrame(writer, frameArray[i]);
        sprintf(filename, "calibrationImages/cam1-%05d.ppm", i+1);
        cvSaveImage(filename, frameArray[i]);
        
        sprintf(filename, "calibrationImages/cam2-%05d.ppm", i+1);
        cvSaveImage(filename, frameArray2[i]);
        
        //cvWriteFrame(writer2, frameArray2[i]);
        printf("Writing frame %d", i);
    }
    
    printf("\n\n");
    
    //cvReleaseVideoWriter(&writer);
    //cvReleaseVideoWriter(&writer2);
    
    cvDestroyWindow("Input");
    cvDestroyWindow("Input2");

    // stop data transmission
    err = dc1394_video_set_transmission(camera, DC1394_OFF);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not stop the camera");
    
    err2 = dc1394_video_set_transmission(camera2, DC1394_OFF);
    DC1394_ERR_CLN_RTN(err2,cleanup_and_exit(camera2),"Could not stop the camera 2");

    // close camera
    cleanup_and_exit(camera);
    cleanup_and_exit(camera2);
    dc1394_free (d);
    dc1394_free (d2);
    
    return 0;
}

int camera_custom_setup(dc1394camera_t *camera)
{
    uint32_t min, max;
    dc1394error_t   err;
    
    /* turn off auto exposure */
    /* turn on the feature - dont know what this means?? */
    err = dc1394_feature_set_power(camera, DC1394_FEATURE_EXPOSURE, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not turn on the exposure feature");
    
    err = dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR_RTN(err,"Could not turn off Auto-exposure");
    
    //err = dc1394_feature_set_value(camera, DC1394_FEATURE_GAMMA, 1);
    //DC1394_ERR_RTN(err, "Count not set gamma.");

    /* get bounds and set */
    err = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_EXPOSURE, &min, &max);
    DC1394_ERR_RTN(err,"Could not get bounds");

    err = dc1394_feature_set_value(camera, DC1394_FEATURE_EXPOSURE, CLAMP(EXPOSURE_VALUE, min, max));
    DC1394_ERR_RTN(err,"Could not set value");
    
    // disable auto-brightness since it is causing issues
    /* turn on the feature - dont know what this means?? */
    err = dc1394_feature_set_power(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_ON);
    DC1394_ERR_RTN(err,"Could not turn on the brightness feature");
    
    /* turn off auto exposure */
    err = dc1394_feature_set_mode(camera, DC1394_FEATURE_BRIGHTNESS, DC1394_FEATURE_MODE_MANUAL);
    DC1394_ERR_RTN(err,"Could not turn off Auto-brightness");

    /* get bounds and set */
    err = dc1394_feature_get_boundaries(camera, DC1394_FEATURE_BRIGHTNESS, &min, &max);
    DC1394_ERR_RTN(err,"Could not get bounds");

    err = dc1394_feature_set_value(camera, DC1394_FEATURE_BRIGHTNESS, CLAMP(BRIGHTNESS_VALUE, min, max));
    DC1394_ERR_RTN(err,"Could not set value");
    
    // --- end brightness --- //
}

