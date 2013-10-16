/*
 * Draws optotrak data to LCMGL
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

#include "../../LCM/lcmt_optotrak_quat.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
   

lcm_t * lcm;

double obstacleXY1[2], obstacleXY2[2], obstacleHeight, obstacleBottom;

bot_lcmgl_t* lcmgl;

// bot frames global
BotFrames *botFrames;

lcmt_optotrak_quat_subscription_t *optotrak_quat_sub;

BotTrans optotrakToLocal, optotrakToLocal2;


static void usage(void)
{
        fprintf(stderr, "usage: optotrak-lcmgl optotrak-quat-channel-name lcmgl-channel-name\n");
        fprintf(stderr, "    optotrak-quat-channel-name : LCM channel name with optotrak QUAT messages\n");
        fprintf(stderr, "    lcmgl-channel-name : LCM channel to publish LCMGL messages on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./optotrak-lcmgl wingeron_x_quat wingeron_x_lcmgl\n");
        fprintf(stderr, "    reads optotrak LCM messages and draws the position using  LCMGL\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_optotrak_quat_unsubscribe(lcm, optotrak_quat_sub);
    lcm_destroy (lcm);

    printf("done.\n");
    
    exit(0);
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

void PlotLcmGlCoordinateTri(BotTrans *trans)
{
    double origin[3];
    origin[0] = 0;
    origin[1] = 0;
    origin[2] = 0;
    
    double originTrans[3];
    
    bot_trans_apply_vec(trans, origin, originTrans);
    
    bot_lcmgl_line_width(lcmgl, 3.0f);
    
    // now get the three edges
    
    // x
    double xEdge[3], xEdgeTrans[3];
    xEdge[0] = 1;
    xEdge[1] = 0;
    xEdge[2] = 0;
    
    bot_trans_apply_vec(trans, xEdge, xEdgeTrans);
    
    //cout << "originTrans: x: " << originTrans[0] << "y: " << originTrans[1] << "z: " << originTrans[2]  << endl;
    
    
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_color3f(lcmgl, 255, 0, 0);
    bot_lcmgl_vertex3f(lcmgl, originTrans[0], originTrans[1], originTrans[2]);
    bot_lcmgl_vertex3f(lcmgl, xEdgeTrans[0], xEdgeTrans[1], xEdgeTrans[2]);
    bot_lcmgl_end(lcmgl);
    
    // y
    double yEdge[3], yEdgeTrans[3];
    yEdge[0] = 0;
    yEdge[1] = 1;
    yEdge[2] = 0;
    
    bot_trans_apply_vec(trans, yEdge, yEdgeTrans);
    
    
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_color3f(lcmgl, 0, 255, 0);
    bot_lcmgl_vertex3f(lcmgl, originTrans[0], originTrans[1], originTrans[2]);
    bot_lcmgl_vertex3f(lcmgl, yEdgeTrans[0], yEdgeTrans[1], yEdgeTrans[2]);
    bot_lcmgl_end(lcmgl);
    
    // z
    double zEdge[3], zEdgeTrans[3];
    zEdge[0] = 0;
    zEdge[1] = 0;
    zEdge[2] = 2;
    
    bot_trans_apply_vec(trans, zEdge, zEdgeTrans);
    
    
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_color3f(lcmgl, 0, 0, 255);
    bot_lcmgl_vertex3f(lcmgl, originTrans[0], originTrans[1], originTrans[2]);
    bot_lcmgl_vertex3f(lcmgl, zEdgeTrans[0], zEdgeTrans[1], zEdgeTrans[2]);
    bot_lcmgl_end(lcmgl);
    
    
}

void optotrak_quat_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_optotrak_quat *msg, void *user)
{

    double xyz[3];
    xyz[0] = msg->x[0]/1000.0;
    xyz[1] = msg->y[0]/1000.0;
    xyz[2] = msg->z[0]/1000.0;
    
    double quat[4];
    quat[0] = msg->q0[0];
    quat[1] = msg->qx[0];
    quat[2] = msg->qy[0];
    quat[3] = msg->qz[0];
    
    //cout << "x = " << xyz[0] << endl;
    
    
    BotTrans newTrans;
    
    bot_trans_set_from_quat_trans(&newTrans, quat, xyz);
    
    bot_frames_get_trans(botFrames, "optotrak", "local", &optotrakToLocal);
    
    //bot_trans_apply_trans(&newTrans, &optotrakToLocal);
    bot_trans_apply_trans(&optotrakToLocal, &newTrans);
    
    //bot_trans_apply_trans(&newTrans, &optotrakToLocal2);
    
    // update transform
    bot_frames_update_frame(botFrames, "optotrak-local", "local", &optotrakToLocal, getTimestampNow());
    
    BotTrans drawTrans;
    bot_frames_get_trans(botFrames, "optotrak-local", "local", &drawTrans);
    
    // draw obstacle
    // publish to LCMGL
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_point_size(lcmgl, 10.5f);
    bot_lcmgl_begin(lcmgl, GL_QUADS);
    
    // obstacle
    bot_lcmgl_color3f(lcmgl, 255, 0, 0);
    bot_lcmgl_vertex3f(lcmgl, obstacleXY1[0], obstacleXY1[1], obstacleBottom);
    bot_lcmgl_vertex3f(lcmgl, obstacleXY1[0], obstacleXY1[1], obstacleBottom + obstacleHeight);
    bot_lcmgl_vertex3f(lcmgl, obstacleXY2[0], obstacleXY2[1], obstacleBottom + obstacleHeight);
    bot_lcmgl_vertex3f(lcmgl, obstacleXY2[0], obstacleXY2[1], obstacleBottom);
    
    bot_lcmgl_end(lcmgl);
    
    // draw coordinate
    
    PlotLcmGlCoordinateTri(&drawTrans);
    
    
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
    
    
    #if 0
    // publish to LCMGL
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_point_size(lcmgl, 10.5f);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
    
    // obstacle
    bot_lcmgl_color3f(lcmgl, 255, 0, 0);
    bot_lcmgl_vertex3f(lcmgl, obstacleXY[0], obstacleXY[1], 0);
    bot_lcmgl_end(lcmgl);
    
    // origin
    bot_lcmgl_begin(lcmgl, GL_POINTS);
    bot_lcmgl_color3f(lcmgl, 0, 0, 255);
    bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]);
    bot_lcmgl_end(lcmgl);
    
    // plane
     
    bot_lcmgl_line_width(lcmgl, 3.0f);
    // x
    
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_color3f(lcmgl, 255, 0, 0);
    bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]);
    bot_lcmgl_vertex3f(lcmgl, xyz[0]+1, xyz[1], xyz[2]);
    bot_lcmgl_end(lcmgl);
    
    // y
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_color3f(lcmgl, 0, 255, 0);
    bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]);
    bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1]+1, xyz[2]);
    bot_lcmgl_end(lcmgl);
    
    // z
    bot_lcmgl_begin(lcmgl, GL_LINE_STRIP);
    bot_lcmgl_color3f(lcmgl, 0, 0, 255);
    bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]);
    bot_lcmgl_vertex3f(lcmgl, xyz[0], xyz[1], xyz[2]+1);
    bot_lcmgl_end(lcmgl);
    
    
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
    #endif
}



int main(int argc,char** argv)
{
    char *channelOptotrakQuat = NULL;
    char *channelLcmGl = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    channelOptotrakQuat = argv[1];
    channelLcmGl = argv[2];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    

    signal(SIGINT,sighandler);
    
    lcmgl = bot_lcmgl_init(lcm, channelLcmGl);
    
    // init obstacle position
    BotParam *param = bot_param_new_from_server(lcm, 0);
    if (param != NULL) {
        if (bot_param_get_double_array(param, "optotrak_obstacle.xy1", obstacleXY1, 2) == -1) {
            fprintf(stderr, "error: unable to get optotrak_obstacle.xy1 from param server\n");
            exit(1); // Don't allow usage without latlon origin.
        } else {
            if (bot_param_get_double_array(param, "optotrak_obstacle.xy2", obstacleXY2, 2) == -1) {
                fprintf(stderr, "error: unable to get optotrak_obstacle.xy1 from param server\n");
            } else {
                if (bot_param_get_double_array(param, "optotrak_obstacle.height", &obstacleHeight, 2) == -1)
                {
                    fprintf(stderr, "error: unable to get optotrak_obstacle.height from param server\n");
                } else {
                    if (bot_param_get_double_array(param, "optotrak_obstacle.bottom", &obstacleBottom, 2) == -1)
                    {
                        fprintf(stderr, "error: unable to get optotrak_obstacle.bottom from param server\n");
                    } else {
            
                        fprintf(stderr, "\n\nInitializing test obstacle at %f, %f --> %f, %f, height: %f, bottom: %f\n", obstacleXY1[0], obstacleXY1[1], obstacleXY2[0], obstacleXY2[1], obstacleHeight, obstacleBottom);
                    }
                }
            }
        }
        
        // init frames
        botFrames = bot_frames_new(lcm, param);
        bot_frames_get_trans(botFrames, "optotrak", "local", &optotrakToLocal);
        bot_frames_get_trans(botFrames, "optotrak-local", "local", &optotrakToLocal2);

    } else {
        fprintf(stderr, "error: no param server, no optotrak_obstacle.xy\n");
        exit(1);
    }
    
    optotrak_quat_sub = lcmt_optotrak_quat_subscribe(lcm, channelOptotrakQuat, &optotrak_quat_handler, NULL);
    
    printf("Receiving:\nOptotrak Quat LCM: %s\nPublishing LCM:\n\tLCMGL: %s\n", channelOptotrakQuat, channelLcmGl);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
