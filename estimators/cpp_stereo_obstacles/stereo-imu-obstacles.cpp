/*
 * Integrates the 3d stereo data (from LCM) and the IMU data (from LCM)
 * and outputs and obstacle map (tbd)
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

#include <Eigen/Dense>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include "../../LCM/lcmt_gps.h"
#include "../../LCM/lcmt_attitude.h"
#include "../../LCM/lcmt_baro_airspeed.h"
#include "../../LCM/lcmt_stereo.h"
    
using Eigen::Matrix3f;
using Eigen::Vector3f;
    
lcm_t * lcm;
char *lcm_out = NULL;
int numFrames = 0;
unsigned long totalTime = 0;

bot_lcmgl_t* lcmgl;

std::mutex attitude_mutex, gps_mutex, baro_airspeed_mutex;

lcmt_stereo_subscription_t * stereo_sub;
lcmt_attitude_subscription_t * attitude_sub;
lcmt_baro_airspeed_subscription_t * baro_airspeed_sub;
lcmt_gps_subscription_t * gps_sub;

// globals for holding state between messages
lcmt_attitude *lastAttitudeMsg;
lcmt_baro_airspeed *lastBaroAirspeedMsg;
lcmt_gps *lastGpsMsg;

bool gpsFlag = false, baroAirspeedFlag = false, attitudeFlag = false;

void rotation_matrix_from_euler(Matrix3f *matrixIn, float roll, float pitch, float yaw);

static void usage(void)
{
        fprintf(stderr, "usage: stereo-imu-obstacles stereo-channel-name attitude-channel-name baro/airspeed-channel-name gps-channel-name\n");
        fprintf(stderr, "    input-channel-name : LCM channel name with stereo LCM messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./stereo-imu-obstacles stereo attitude baro-airspeed gps\n");
}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    lcmt_attitude_unsubscribe(lcm, attitude_sub);
    lcmt_gps_unsubscribe(lcm, gps_sub);
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
    if (!attitudeFlag || !gpsFlag || !baroAirspeedFlag)
    {
        // we don't have all the data we need yet, bail out
        return;
    }
    
    if (lastStereo == NULL)
    {
        lastStereo = lcmt_stereo_copy(msg);
    }
    
    msg = lastStereo;

    // start the rate clock
    struct timeval start, now;
    unsigned long elapsed;
    gettimeofday( &start, NULL );
    
    // on the stereo handler is when we trigger a new update for something
    // this must last less than 8.3ms, which is the rate of the stereo data.
    
    //cout << "got stereo message" << endl;
    
    // we need to be careful of threading issues if other data comes in while we're operating here
    
    // lock all the mutexes
    // this shouldn't take a long time since each handler only copies some data when it has the mutexes.
    // mostly this is here to prevent data from changing under our feet as we go
    attitude_mutex.lock();
    gps_mutex.lock();
    baro_airspeed_mutex.lock();
    
    // ok, we're ready for the real processing now...
    
    // first, rotate the stereo points into the global frame
    
    // build the rotation matrix
    Matrix3f rotationMatrix;
    rotation_matrix_from_euler(&rotationMatrix, lastAttitudeMsg->roll, lastAttitudeMsg->pitch, lastAttitudeMsg->yaw);
    
    
    Matrix3f toOpengl;
    toOpengl << (Matrix3f() << -1, 0, 0, 0, -1, 0, 0, 0, 1).finished();
    
    // begin opengl
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_point_size(lcmgl, 10.5f);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
    
    // make a blue point at the origin
    bot_lcmgl_color3f(lcmgl, 0, 0, 1);
    bot_lcmgl_vertex3f(lcmgl, 0, 0, 0);
    
    bot_lcmgl_color3f(lcmgl, 0.5, 0.5, 0.5);
    // now apply this matrix to each point
    for (int i = 0; i<msg->number_of_points; i++)
    {
        Vector3f thisPoint;
        thisPoint << msg->z[i], msg->x[i], msg->y[i];
        
        //cout << endl << "-------------" << endl << thisPoint << endl << "--------------" << endl;
        
        Vector3f transformedPoint = toOpengl*rotationMatrix*thisPoint;
        
        bot_lcmgl_vertex3f(lcmgl, transformedPoint(0), -transformedPoint(1), transformedPoint(2));
        bot_lcmgl_vertex3f(lcmgl, 1, -1, 0);
    }
    
    //cout << lastAttitudeMsg->roll << lastAttitudeMsg->pitch << lastAttitudeMsg->yaw << endl;
    
    //cout << rotationMatrix << endl;
    
    bot_lcmgl_end(lcmgl);
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
    
    // we're done, unlock everything
    attitude_mutex.unlock();
    gps_mutex.unlock();
    baro_airspeed_mutex.unlock();
    
    numFrames ++;
    
    // compute framerate
    gettimeofday( &now, NULL );
    
    elapsed = (now.tv_usec / 1000 + now.tv_sec * 1000) - (start.tv_usec / 1000 + start.tv_sec * 1000);
    totalTime += elapsed;
    printf("\r%d frames | %f ms/frame", numFrames, (float)totalTime/numFrames);
    fflush(stdout);
}

void attitude_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_attitude *msg, void *user)
{
    // get the lock
    attitude_mutex.lock();
    lastAttitudeMsg = lcmt_attitude_copy(msg);
    attitude_mutex.unlock();
    attitudeFlag = true;
}

void baro_airspeed_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_baro_airspeed *msg, void *user)
{
    baro_airspeed_mutex.lock();
    lastBaroAirspeedMsg = lcmt_baro_airspeed_copy(msg);
    baro_airspeed_mutex.unlock();
    
    baroAirspeedFlag = true;
}

void gps_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_gps *msg, void *user)
{
    gps_mutex.lock();
    lastGpsMsg = lcmt_gps_copy(msg);
    gps_mutex.unlock();
    
    gpsFlag = true;
}


int main(int argc,char** argv)
{
    char *channelStereo = NULL;
    char *channelAttitude = NULL;
    char *channelBaroAirspeed = NULL;
    char *channelGps = NULL;

    if (argc!=5) {
        usage();
        exit(0);
    }

    channelStereo = argv[1];
    channelAttitude = argv[2];
    channelBaroAirspeed = argv[3];
    channelGps = argv[4];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    stereo_sub =  lcmt_stereo_subscribe (lcm, channelStereo, &stereo_handler, NULL);
    attitude_sub =  lcmt_attitude_subscribe (lcm, channelAttitude, &attitude_handler, NULL);
    baro_airspeed_sub =  lcmt_baro_airspeed_subscribe (lcm, channelBaroAirspeed, &baro_airspeed_handler, NULL);
    gps_sub =  lcmt_gps_subscribe (lcm, channelGps, &gps_handler, NULL);

    lcmgl = bot_lcmgl_init(lcm, "lcmgl-stereo-transformed");

    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tStereo: %s\n\tAttitude: %s\n\tBarometric altitude and airspeed: %s\n\tGPS: %s\n", channelStereo, channelAttitude, channelBaroAirspeed, channelGps);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}


/* *** from Ardupilot source (so it will be consistent with their angle convention *** */
// ================================================================================== //
// create a rotation matrix given some euler angles
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void rotation_matrix_from_euler(Matrix3f *matrix, float roll, float pitch, float yaw)
{
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    // [ a.x    a.y     a.z ]
    // [ b.x    b.y     b.z ]
    // [ c.x    c.y     c.z ]
    (*matrix)(0,0) = cp * cy;
    (*matrix)(0,1) = (sr * sp * cy) - (cr * sy);
    (*matrix)(0,2) = (cr * sp * cy) + (sr * sy);
    (*matrix)(1,0) = cp * sy;
    (*matrix)(1,1) = (sr * sp * sy) + (cr * cy);
    (*matrix)(1,2) = (cr * sp * sy) - (sr * cy);
    (*matrix)(2,0) = -sp;
    (*matrix)(2,1) = sr * cp;
    (*matrix)(2,2) = cr * cp;
}
#if 0
// calculate euler angles from a rotation matrix
// this is based on http://gentlenav.googlecode.com/files/EulerAngles.pdf
void Matrix3f::to_euler(float *roll, float *pitch, float *yaw)
{
    if (pitch != NULL) {
        *pitch = -safe_asin(c.x);
    }
    if (roll != NULL) {
        *roll = atan2f(c.y, c.z);
    }
    if (yaw != NULL) {
        *yaw = atan2f(b.x, a.x);
    }
}
#endif
// ================================================================================== //
