/*
 * Draws obstacles found in plane.cfg
 *
 * Author: Amruth Venkatraman, <amruthv@mit.edu> 2013
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
#include <boost/lexical_cast.hpp>

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>


lcm_t * lcm;

bot_lcmgl_t* lcmgl;

void sighandler(int dum)
{
    printf("\nClosing... ");

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

void usage() {
    fprintf(stderr, "provide channel name to publish lcmgl data to as argument.");
}

int main(int argc,char** argv)
{
    int numObstacles;
    char *channelLcmGl = NULL;
    
    if (argc != 2) {
        usage();
        exit(0);
    }

    channelLcmGl = argv[1];

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
    if (param == NULL) {
        fprintf(stderr, "error: no param server! :(\n");
        exit(1);
    }
    //bot_param_get_int(param, "obstacles.numObstacles", &numObstacles);
    //printf("num Obstacles: %d", numObstacles);
    //Else there is a param server
    if (bot_param_get_int(param, "obstacles.numObstacles", &numObstacles) == -1) {
        fprintf(stderr, "error: No number of obstacles stated.\n");
    }
    //Assume obstacles are listed as obstacles.obstacle1, obstacles.obstacle2, etc..
    bot_lcmgl_push_matrix(lcmgl);
    bot_lcmgl_color3f(lcmgl, 255, 0, 0);
    
    std::string obstaclePrefix = "obstacles.obstacle";
    std::string obstacle;
    for (int i=1; i <= numObstacles; i++) {
        obstacle = obstaclePrefix + boost::lexical_cast<std::string>(i);
        double xPos, yPos, bottom, height, radius;
        std::string configXParam, configYParam, configBotParam, configHeightParam, configRParam;
        configXParam = obstacle + ".x";
        configYParam = obstacle + ".y";
        configBotParam = obstacle + ".bottom";
        configHeightParam = obstacle + ".height";
        configRParam = obstacle + ".radius";
        
        bot_param_get_double(param, configXParam.c_str(), &xPos);
        bot_param_get_double(param, configYParam.c_str(), &yPos);
        bot_param_get_double(param, configBotParam.c_str(), &bottom);
        bot_param_get_double(param, configHeightParam.c_str(), &height);
        bot_param_get_double(param, configRParam.c_str(), &radius);
        double xyz[] = {xPos,yPos, bottom + height/2};
        float dims[] = {radius*2, radius*2, height};
        bot_lcmgl_begin(lcmgl, GL_QUADS);
        bot_lcmgl_box(lcmgl, xyz, dims); 
        bot_lcmgl_end(lcmgl); 
    }
    
    bot_lcmgl_pop_matrix(lcmgl);
    bot_lcmgl_switch_buffer(lcmgl);
    
    return 0;
}
