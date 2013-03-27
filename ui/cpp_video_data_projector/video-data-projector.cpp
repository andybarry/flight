/*
 * Projects trajectories and stereo collections points onto recorded (or live) video
 * when it receives LCM messages (assumes you'll play them back for logging)
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include "../../LCM/lcmt_stereo.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>

#include "mav_ins_t.h" // from Fixie
#include "mav_gps_data_t.h" // from Fixie


using namespace std;
using namespace cv;using namespace std;
using namespace cv;

#define GRAVITY_MSS 9.80665f // this matches the ArduPilot definition

lcm_t * lcm;

lcmt_stereo_subscription_t * stereo_sub;

VideoCapture videoFileCap;


// global calibration
Mat camMatL, dMatL, camMatR, dMatR;


static void usage(void)
{
        fprintf(stderr, "usage: video-data-projector video-file stereo-lcm-channel-name\n");
        fprintf(stderr, "    video-file: .avi file with video\n");
        fprintf(stderr, "    stereo-lcm-channel-name : LCM channel to receive stereo matches on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    video-data-projector videoL-2013-03-27-18-53-43.avi stereo\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    lcm_destroy (lcm);
    
    waitKey(1);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

void Get3DPointsFromStereoMsg(const lcmt_stereo *msg, vector<Point3f> *pointsOut)
{
    for (int i=0; i<msg->number_of_points; i++)
    {
        pointsOut->push_back(Point3f(msg->x[i], msg->y[i], msg->z[i]));
    }
}

void Draw3DPointsOnImage(Mat cameraImage, vector<Point3f> *pointsListIn, Scalar color)
{
    vector<Point3f> &pointsList = *pointsListIn;
    
    if (pointsList.size() <= 0)
    {
        cout << "zero sized points list" << endl;
        return;
    }
    
    int color = 127;
    
    vector<Point2f> imgPointsList;

    projectPoints(pointsList, Mat::zeros(3, 1, CV_32F), Mat::zeros(3, 1, CV_32F), camMatL, dMatL, imgPointsList);
    
    // now draw the points onto the image
    for (int i=0; i<int(imgPointsList.size()); i++)
    {
        rectangle(cameraImage, Point(imgPointsList[i].x - 2, imgPointsList[i].y - 2),
            Point(imgPointsList[i].x + 2, imgPointsList[i].y + 2), color);
    }
}

void Get3DPointsFromTrajMsg(const lcmt_traj_points *msg, vector<Point3f> trajPonts)
{
    
}

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
    // if we got a stereo message, grab the next frame of video
    Mat thisImg;
    
    videoFileCap >> thisImg;
    
    // project points from 3D to 2D based on our calibration
    vector<Point3f> stereoPoints;
    Get3DPointsFromStereoMsg(msg, &stereoPoints);
    
    Draw3DPointsOnImage(thisImg, &stereoPoints, Scalar(0, 0, 255));
    
    vector<Point3f> trajPoints;

    Get3DPontsFromTrajMsg(lastTrajMsg, &trajPoints);
    DrawTrajectoryOnImage(thisImg, &trajectoryPoints, Scalar(255, 0, 0));


    imshow("Data on video", thisImg);
    
    waitKey(1); // must do a waitKey to get GUI events to be called
}


int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *videoFile = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    videoFile = argv[1];
    channelStereo = argv[2];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    stereo_sub = lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);

    signal(SIGINT,sighandler);

    printf("Reading:\n\tVideo file: %s\n\tStereo LCM: %s\n", videoFile, channelStereo);
    
    // open the video file
    if (videoFileCap.open(videoFile) == true)
    {
        cout << "File opened successfully." << endl;
    } else {
        cout << "ERROR: failed to open video file: " << string(videoFile) << endl;
        exit(1);
    }
    
    // load calibration
    cout << "Loading calibration..." << endl;
    
    
    
    CvMat *m1 = (CvMat *)cvLoad("../../sensors/stereo/calib/M1.xml",NULL,NULL,NULL);
    CvMat *d1 = (CvMat *)cvLoad("../../sensors/stereo/calib/D1.xml",NULL,NULL,NULL);
    CvMat *m2 = (CvMat *)cvLoad("../../sensors/stereo/calib/M2.xml",NULL,NULL,NULL);
    CvMat *d2 = (CvMat *)cvLoad("../../sensors/stereo/calib/D2.xml",NULL,NULL,NULL);
    
    camMatL = Mat(m1, true);
    dMatL = Mat(d1,true);
    camMatR = Mat(m2,true);
    dMatR = Mat(d2,true);
    
    
    namedWindow("Data on video", CV_WINDOW_AUTOSIZE);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
        waitKey(1);
    }

    return 0;
}
