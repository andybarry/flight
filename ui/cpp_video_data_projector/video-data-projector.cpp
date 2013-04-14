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
#include <mutex>

#include "opencv2/opencv.hpp"

#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_reprojected.h"
#include "../../LCM/lcmt_trajectory_number.h"
#include "../../controllers/cpp_stereo_obstacles/Trajectory.hpp"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>

#include "mav_ins_t.h" // from Fixie
#include "mav_gps_data_t.h" // from Fixie


using namespace std;
using namespace cv;

#define GRAVITY_MSS 9.80665f // this matches the ArduPilot definition

lcm_t * lcm;

lcmt_stereo_subscription_t * stereo_sub;
lcmt_trajectory_number_subscription_t * trajnum_sub;

VideoCapture videoFileCap;

lcmt_trajectory_number *lastTrajMsg;

std::mutex trajnumMutex;

// bot frames global
BotFrames *botFrames;


// global calibration
Mat camMatL, dMatL, camMatR, dMatR;

char *channelReprojected = NULL;


static void usage(void)
{
        fprintf(stderr, "usage: video-data-projector video-file stereo-lcm-channel-name trajectory-number-lcm-channel-name [projected-channel]\n");
        fprintf(stderr, "    video-file: .avi file with video\n");
        fprintf(stderr, "    stereo-lcm-channel-name : LCM channel to receive stereo matches on\n");
        fprintf(stderr, "    reprojected channel (optional): if passed, will publish LCM messages with image-based x, y, and depth data.\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./video-data-projector videoL-2013-03-27-18-53-43.avi stereo trajectory_number\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    lcmt_trajectory_number_unsubscribe(lcm, trajnum_sub);
    
    lcm_destroy (lcm);
    
    // let opencv close it's windows
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

void PublishReprojectedPoints(vector<Point2f> imgPointsList, vector<Point3f> *pointsListIn, int width, int height)
{
    // pointsList contains the (x,y,z) position of the points, which we need
    // because we want the depth as well as pixel location.    
    vector<Point3f> &pointsList = *pointsListIn;
    
    // check to see if the channel is defined
    if (channelReprojected == NULL)
    {
        return;
    }
    
    // create the lcm message
    lcmt_stereo_reprojected msg;
    
    msg.number_of_points = int(imgPointsList.size());
    
    // create the message's arrays
    int x[msg.number_of_points];
    int y[msg.number_of_points];
    float depth[msg.number_of_points];
    
    // populate it with points from imgPointsList
    for (int i=0; i<int(imgPointsList.size()); i++)
    {
        x[i] = imgPointsList[i].x;
        y[i] = imgPointsList[i].y;
        
        depth[i] = -pointsList[i].z/10.0; // coming in in cm
    }
    
    // fill in the message's fields
    msg.x = x;
    msg.y = y;
    msg.depth = depth;
    
    msg.width = width;
    msg.height = height;
    
    // now publish the message
    lcmt_stereo_reprojected_publish(lcm, channelReprojected, &msg);
}

// this function also publishes the image points on
// channelReprojected if that variable is not NULL
void Draw3DPointsOnImage(Mat cameraImage, vector<Point3f> *pointsListIn, Scalar color)
{
    vector<Point3f> &pointsList = *pointsListIn;
    
    if (pointsList.size() <= 0)
    {
        cout << "Draw3DPointsOnimage: zero sized points list" << endl;
        return;
    }
    
    vector<Point2f> imgPointsList;

    projectPoints(pointsList, Mat::zeros(3, 1, CV_32F), Mat::zeros(3, 1, CV_32F), camMatL, dMatL, imgPointsList);
    
    // now draw the points onto the image
    for (int i=0; i<int(imgPointsList.size()); i++)
    {
        rectangle(cameraImage, Point(imgPointsList[i].x - 2, imgPointsList[i].y - 2),
            Point(imgPointsList[i].x + 2, imgPointsList[i].y + 2), color, CV_FILLED);
    }
    
    // publish points
    PublishReprojectedPoints(imgPointsList, pointsListIn, cameraImage.cols, cameraImage.rows);
}

void Get3DPointsFromTrajMsg(const lcmt_trajectory_number *msg, vector<Point3f> *trajPoints)
{
    if (msg == NULL)
    {
        return;
    }
    
    // load the 3D points from a file
    
    string filename = "../../controllers/cpp_stereo_obstacles/trajlib/trajlib" + to_string((long long int) msg->trajNum) + ".csv";
    
    Trajectory thisTraj(filename, true);
    
    BotTrans toOpenCv;
    bot_frames_get_trans(botFrames, "local", "opencvFrame", &toOpenCv);
    
    
    // dump the ponits into trajPoints
    for (int i=0; i<int(thisTraj.xpoints.size()); i++)
    {
        double thisPoint[3];
        double newPoint[3];
        thisPoint[0] = thisTraj.xpoints[i][0];
        thisPoint[1] = thisTraj.xpoints[i][1];
        thisPoint[2] = thisTraj.xpoints[i][2];
        
        bot_trans_apply_vec(&toOpenCv, thisPoint, newPoint);
        
        trajPoints->push_back(Point3f(newPoint[0]/10.0, newPoint[1]/10.0, newPoint[2]/10.0));
    }
    
}

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
    trajnumMutex.lock();
    
    // if we got a stereo message, grab the next frame of video
    Mat thisImg;
    
    videoFileCap >> thisImg;
    
    // project points from 3D to 2D based on our calibration
    vector<Point3f> stereoPoints;
    Get3DPointsFromStereoMsg(msg, &stereoPoints);
    
    Draw3DPointsOnImage(thisImg, &stereoPoints, Scalar(0, 0, 255));
    
    vector<Point3f> trajPoints;

    // do not draw trajectories if we're reprojecting images
    if (channelReprojected == NULL)
    {
        Get3DPointsFromTrajMsg(lastTrajMsg, &trajPoints);
        Draw3DPointsOnImage(thisImg, &trajPoints, Scalar(255, 0, 0));
    }
    
    trajnumMutex.unlock();


    imshow("Data on video", thisImg);
    
    waitKey(1); // must do a waitKey to get GUI events to be called
}

void trajnum_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_trajectory_number *msg, void *user)
{
    trajnumMutex.lock();
    lastTrajMsg = lcmt_trajectory_number_copy(msg);
    trajnumMutex.unlock();
}

int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *channelTrajNum = NULL;
    char *videoFile = NULL;
    
    if (argc!=4 && argc!=5) {
        usage();
        exit(0);
    }

    videoFile = argv[1];
    channelStereo = argv[2];
    channelTrajNum = argv[3];
    
    if (argc == 5)
    {
        channelReprojected = argv[4];
    }

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    stereo_sub = lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);
    trajnum_sub = lcmt_trajectory_number_subscribe(lcm, channelTrajNum, &trajnum_handler, NULL);

    signal(SIGINT,sighandler);

    printf("Reading:\n\tVideo file: %s\n\tStereo LCM: %s\n\tTrajectory Number LCM: %s\n", videoFile, channelStereo, channelTrajNum);
    
    if (channelReprojected != NULL)
    {
        printf("Publishing reprojected stereo: %s\n", channelReprojected);
    }
    
    // open the video file
    if (videoFileCap.open(videoFile) == true)
    {
        cout << "File opened successfully." << endl;
    } else {
        cout << "ERROR: failed to open video file: " << string(videoFile) << endl;
        exit(1);
    }
    
    // init frames
    BotParam *param = bot_param_new_from_server(lcm, 0);
    botFrames = bot_frames_new(lcm, param);
    
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
    
    cout << "Calibration loaded successfully." << endl;
    
    
    namedWindow("Data on video", CV_WINDOW_AUTOSIZE);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
        waitKey(1);
    }

    return 0;
}
