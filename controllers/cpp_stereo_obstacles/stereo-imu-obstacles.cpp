/*
 * Integrates the 3d stereo data (from LCM) and the IMU data (from LCM)
 * and generates an obstacle map.  Then uses that map with a trajectory
 * library for control.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>


using namespace std;


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <mutex>
#include <math.h>

#include "opencv2/opencv.hpp"

#include <Eigen/Dense>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcmtypes/mav_pose_t.h>
#include <lcmtypes/octomap_raw_t.h>

#include <octomap/OcTree.h>

#include "../../LCM/lcmt_stereo.h"

#include "TrajectoryLibrary.hpp"
#include "Trajectory.hpp"

    
#define IMAGE_GL_Y_OFFSET 400
#define IMAGE_GL_WIDTH 376
#define IMAGE_GL_HEIGHT 240
    

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace cv;
using namespace std;
using namespace octomap;

lcm_t * lcm;
char *lcm_out = NULL;
int numFrames = 0;
unsigned long totalTime = 0;

// global octree
OcTree octree(0.1);

// global trajectory library
TrajectoryLibrary trajlib;

bot_lcmgl_t* lcmgl;

// bot frames global
BotFrames *botFrames;

// global mutex
std::mutex pose_mutex;


// globals for subscription functions, so we can unsubscribe in the control-c handler
lcmt_stereo_subscription_t * stereo_sub;
mav_pose_t_subscription_t * pose_sub;

// globals for holding state between messages
mav_pose_t *lastPoseMsg;

// globals for ensuring data has arrived
bool poseFlag = false;

static void usage(void)
{
        fprintf(stderr, "usage: stereo-imu-obstacles stereo-channel-name pose-channel-name trajectory-library-directory\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with stereo LCM messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./stereo-imu-obstacles stereo STATE_ESTIMATOR_POSE trajlib\n");
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    mav_pose_t_unsubscribe(lcm, pose_sub);
    lcm_destroy (lcm);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000.0) + (float)thisTime.tv_usec/1000.0 + 0.5;
}

lcmt_stereo *lastStereo = NULL;

void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
    if (!poseFlag)
    {
        // we don't have all the data we need yet, bail out
        return;
    }
    
    if (lastStereo == NULL)
    {
        lastStereo = lcmt_stereo_copy(msg);
    }
    
    //msg = lastStereo;

    // start the rate clock
    struct timeval start, now;
    unsigned long elapsed;
    gettimeofday( &start, NULL );
    
    // on the stereo handler is when we trigger a new update for something
    // this must last less than 8.3ms, which is the rate of the stereo data.
    
    // we need to be careful of threading issues if other data comes in while we're operating here
    
    // lock all the mutexes
    // this shouldn't take a long time since each handler only copies some
    // data when it has the mutexes.
    
    // mostly this is here to prevent data from changing under our feet as we go
    pose_mutex.lock();
    
    // ok, we're ready for the real processing now...
    
    // first, rotate the stereo points into the global frame
    
    // build the rotation matrix
    
    
    double rotMat[9];
    bot_quat_to_matrix(lastPoseMsg->orientation, rotMat);
    
    Eigen::Matrix3d rotationMatrix(rotMat);
    
    //cout << rotationMatrix << endl;
    
    Matrix3d toOpengl;
    toOpengl << (Matrix3d() << -1, 0, 0, 0, -1, 0, 0, 0, 1).finished();
    
    vector<Point3d> opencvPoints;
    
    Vector3d posVec(lastPoseMsg->pos);
    
    // get transform from global to local frame
    BotTrans toOpenCv, bodyToLocal;
    bot_frames_get_trans(botFrames, "opencvFrame", "local", &toOpenCv);
    bot_frames_get_trans(botFrames, "body", "local", &bodyToLocal);
    
    // get the new origin
    double origin[3];
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    
    double planeOrigin[3];
    bot_trans_apply_vec(&bodyToLocal, origin, planeOrigin);
    octomath::Vector3 vecPlaneOrigin(planeOrigin[0], planeOrigin[1], planeOrigin[2]);
    
    // begin opengl
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_point_size(lcmgl, 5.5f);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
    
    // make a blue point at the origin
    bot_lcmgl_color3f(lcmgl, 0, 0, 1);
    bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
    
    bot_lcmgl_color3f(lcmgl, 0.5, 0.5, 0.5);
    
    // now apply this matrix to each point
    for (int i = 0; i<msg->number_of_points; i++)
    {
        Vector3d thisPoint;
        thisPoint << msg->z[i], msg->x[i], msg->y[i];
        double thisPointd[3];
        double transPoint[3];
        thisPointd[0] = msg->x[i]/10;
        thisPointd[1] = msg->y[i]/10;
        thisPointd[2] = -msg->z[i]/10; // opencv sends the z coordinate reversed from what we expect and is also in cm
        
        //cout << endl << "-------------" << endl << thisPoint << endl << "--------------" << endl;
        
        // add the position vector
        //Vector3d transformedPoint = thisPoint + posVec;
        bot_trans_apply_vec(&toOpenCv, thisPointd, transPoint);
        
        Vector3d transformedPoint(transPoint);
        octomath::Vector3 vecNewPoint(transPoint[0], transPoint[1], transPoint[2]);
        
        // now rotate
        //transformedPoint = toOpengl*rotationMatrix*thisPoint;
        //transformedPoint = toOpengl*transformedPoint;
        
        float grey = msg->grey[i]/255.0;
        
        bot_lcmgl_color3f(lcmgl, grey, grey, grey);
        //bot_lcmgl_vertex3f(lcmgl, transformedPoint(0), -transformedPoint(1), transformedPoint(2));
        bot_lcmgl_vertex3f(lcmgl, transformedPoint(0), transformedPoint(1), transformedPoint(2));
        
        // add this point to the octree
        //cout << "Inserted ray: " << vecPlaneOrigin << " --> " << vecNewPoint << endl;
        octree.insertRay(vecPlaneOrigin, vecNewPoint);
        
        
        opencvPoints.push_back(Point3f(msg->x[i], msg->y[i], msg->z[i]));
    }
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_pop_matrix(lcmgl);
    
    // only publish every so often since it swamps the viewer
    if (numFrames%30 == 0)
    {
        // publish octomap to LCM
        octomap_raw_t ocMsg;
        ocMsg.utime = getTimestampNow();

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                ocMsg.transform[i][j] = 0;
            }
            ocMsg.transform[i][i] = 1;
        }

        std::stringstream datastream;
        octree.writeBinaryConst(datastream);
        std::string datastring = datastream.str();
        ocMsg.data = (uint8_t *) datastring.c_str();
        ocMsg.length = datastring.size();

        octomap_raw_t_publish(lcm, "OCTOMAP", &ocMsg);
    }   
       
    //if (numFrames%200 == 0)
    {
        bot_lcmgl_switch_buffer(lcmgl);
    }
    

    // we're done, unlock everything
    pose_mutex.unlock();
    
    numFrames ++;
    
    // compute framerate
    gettimeofday( &now, NULL );
    
    elapsed = (now.tv_usec / 1000 + now.tv_sec * 1000) - (start.tv_usec / 1000 + start.tv_sec * 1000);
    totalTime += elapsed;
    //printf("\r%d frames | %f ms/frame", numFrames, (float)totalTime/numFrames);
    //fflush(stdout);
}

// Handler for pose LCM messages
void pose_handler(const lcm_recv_buf_t *rbuf, const char* channel, const mav_pose_t *msg, void *user)
{
    pose_mutex.lock();
    lastPoseMsg = mav_pose_t_copy(msg);
    pose_mutex.unlock();
    
    poseFlag = true;
}


int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *channelPose = NULL;
    char *libDir = NULL;

    if (argc!=4) {
        usage();
        exit(0);
    }

    channelStereo = argv[1];
    channelPose = argv[2];
    libDir = argv[3];
    

    lcm = lcm_create ("udpm://239.255.76.68:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    stereo_sub =  lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);
    pose_sub =  mav_pose_t_subscribe (lcm, channelPose, &pose_handler, NULL);

    lcmgl = bot_lcmgl_init(lcm, "lcmgl-stereo-transformed");
    bot_lcmgl_enable(lcmgl, GL_BLEND);
    
    // init frames
    BotParam *param = bot_param_new_from_server(lcm, 0);
    botFrames = bot_frames_new(lcm, param);
    
    // init trajectory library
    trajlib.LoadLibrary(libDir);
    
    Trajectory temptraj;
    trajlib.FindFarthestTrajectory(octree, &temptraj);
    
    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tStereo: %s\n\tPose: %s\n", channelStereo, channelPose);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
