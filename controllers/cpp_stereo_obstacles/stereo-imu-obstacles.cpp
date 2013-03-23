/*
 * Integrates the 3d stereo data (from LCM) and the IMU data (from LCM)
 * and generates an obstacle map.  Then uses that map with a trajectory
 * library for control.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include "stereo-imu-obstacles.hpp"

    
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


// globals for subscription functions, so we can unsubscribe in the control-c handler
lcmt_stereo_subscription_t * stereo_sub;


lcmt_stereo *lastStereo = NULL;

static void usage(void)
{
        fprintf(stderr, "usage: stereo-imu-obstacles stereo-channel-name trajectory-library-directory\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with stereo LCM messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./stereo-imu-obstacles stereo trajlib\n");
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
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


void stereo_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user)
{
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
    
    // get transform from global to body frame
    BotTrans toOpenCv, bodyToLocal;
    bot_frames_get_trans(botFrames, "opencvFrame", "local", &toOpenCv);
    bot_frames_get_trans(botFrames, "body", "local", &bodyToLocal);
    
    
    // insert the points into the octree
    InsertPointsIntoOctree(msg, &toOpenCv, &bodyToLocal);
    
    // search the trajectory library for the best trajectory
    Trajectory *farthestTraj;
    trajlib.FindFarthestTrajectory(&octree, farthestTraj, &bodyToLocal, lcmgl);
    
    
    // only publish every so often since it swamps the viewer
    if (numFrames%30 == 0)
    {
        PublishOctomap();
    }   

    numFrames ++;
    
    // compute framerate
    gettimeofday( &now, NULL );
    
    elapsed = (now.tv_usec / 1000 + now.tv_sec * 1000) - (start.tv_usec / 1000 + start.tv_sec * 1000);
    totalTime += elapsed;
    //printf("\r%d frames | %f ms/frame", numFrames, (float)totalTime/numFrames);
    //fflush(stdout);
}

void InsertPointsIntoOctree(const lcmt_stereo *msg, BotTrans *toOpenCv, BotTrans *bodyToLocal)
{
    // get the new origin
    double origin[3];
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    
    double planeOrigin[3];
    bot_trans_apply_vec(bodyToLocal, origin, planeOrigin);
    octomath::Vector3 vecPlaneOrigin(planeOrigin[0], planeOrigin[1], planeOrigin[2]);
    
    
    // now apply this matrix to each point
    for (int i = 0; i<msg->number_of_points; i++)
    {
        Vector3d thisPoint;
        thisPoint << msg->z[i], msg->x[i], msg->y[i];
        double thisPointd[3];
        double transPoint[3];
        thisPointd[0] = msg->x[i]/10;
        thisPointd[1] = msg->y[i]/10;
        
        // opencv sends the z coordinate reversed from what we expect and is also in cm
        thisPointd[2] = -msg->z[i]/10;
        
        // add the position vector
        bot_trans_apply_vec(toOpenCv, thisPointd, transPoint);
        
        octomath::Vector3 vecNewPoint(transPoint[0], transPoint[1], transPoint[2]);
        
        // add this point to the octree
        octree.insertRay(vecPlaneOrigin, vecNewPoint);
        
    }
}

void PublishOctomap()
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


int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *libDir = NULL;

    if (argc!=3) {
        usage();
        exit(0);
    }

    channelStereo = argv[1];
    libDir = argv[2];
    

    lcm = lcm_create ("udpm://239.255.76.68:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    stereo_sub =  lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);

    lcmgl = bot_lcmgl_init(lcm, "lcmgl-stereo-transformed");
    bot_lcmgl_enable(lcmgl, GL_BLEND);
    
    // init frames
    BotParam *param = bot_param_new_from_server(lcm, 0);
    botFrames = bot_frames_new(lcm, param);
    
    // init trajectory library
    trajlib.LoadLibrary(libDir);
    
    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tStereo: %s\n", channelStereo);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
