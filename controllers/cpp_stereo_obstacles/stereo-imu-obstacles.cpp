/*
 * Integrates the 3d stereo data (from LCM) and the IMU data (from LCM)
 * and generates an obstacle map.  Then uses that map with a trajectory
 * library for control.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2014
 *
 */

#include "stereo-imu-obstacles.hpp"

    
#define IMAGE_GL_Y_OFFSET 400
#define IMAGE_GL_WIDTH 376
#define IMAGE_GL_HEIGHT 240
    
//#define OCTREE_LIFE 2000000 // in usec
#define OCTREE_LIFE 2000 // in msec

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;
using namespace octomap;

lcm_t * lcm;
char *lcm_out = NULL;
int numFrames = 0;
unsigned long totalTime = 0;

int64_t currentOctreeTimestamp, buildingOctreeTimestamp;

// global trajectory library
TrajectoryLibrary trajlib;

bot_lcmgl_t* lcmgl;


// globals for subscription functions, so we can unsubscribe in the control-c handler
lcmt_stereo_subscription_t * stereo_sub;


lcmt_stereo *lastStereo = NULL;




void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    
    StereoOctomap *octomap = (StereoOctomap*)user;
    
    if (lastStereo == NULL)
    {
        lastStereo = lcmt_stereo_copy(msg);
    }
    
    //msg = lastStereo;

    // start the rate clock
    struct timeval start, now;
    unsigned long elapsed;
    gettimeofday( &start, NULL );
    
    octomap->ProcessStereoMessage(msg);
    
    
    // search the trajectory library for the best trajectory
    /*
    Trajectory* farthestTraj = trajlib.FindFarthestTrajectory(currentOctree, &bodyToLocal, lcmgl);
    
    // publish the farthest trajectory number over LCM for visualization
    lcmt_trajectory_number trajNumMsg;
    trajNumMsg.timestamp = getTimestampNow();
    trajNumMsg.trajNum = farthestTraj->GetTrajectoryNumber();
    
    lcmt_trajectory_number_publish(lcm, "trajectory_number", &trajNumMsg);
    
    
    // only publish every so often since it swamps the viewer
    if (numFrames%30 == 0)
    {
        PublishOctomap();
    }   
    */
    numFrames ++;
    
    // compute framerate
    gettimeofday( &now, NULL );
    
    elapsed = (now.tv_usec / 1000 + now.tv_sec * 1000) - (start.tv_usec / 1000 + start.tv_sec * 1000);
    totalTime += elapsed;
    printf("\r%d frames | %f ms/frame", numFrames, (float)totalTime/numFrames);
    fflush(stdout);
}

int main(int argc,char** argv)
{
    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    // init frames
    BotParam *param = bot_param_new_from_server(lcm, 0);
    BotFrames *bot_frames = bot_frames_new(lcm, param);
    
    // init octomap
    StereoOctomap octomap(bot_frames);

    char *stereo_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo", &stereo_channel) >= 0) {
        stereo_sub = lcmt_stereo_subscribe(lcm, stereo_channel, &stereo_handler, &octomap);
    }

    lcmgl = bot_lcmgl_init(lcm, "lcmgl-stereo-transformed");
    bot_lcmgl_enable(lcmgl, GL_BLEND);
    

    
    
    
    // init trajectory library
    //trajlib.LoadLibrary(libDir);
    
    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tStereo: %s\n", stereo_channel);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    lcm_destroy (lcm);
    
    printf("done.\n");
    
    exit(0);
}
