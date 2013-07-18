/*
 * Displays process and timesync information on LCMGL
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

#include "../../LCM/lcmt_process_control.h"
#include "../../LCM/lcmt_log_size.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
   
   

lcm_t * lcm;

mutex mux;


lcmt_process_control_subscription_t *process_sub;
lcmt_log_size_subscription_t *log_size_sub;


struct StringsOutStruct {
    string time;
    string logfilesize;
};

StringsOutStruct stringsOut;



static void usage(void)
{
        fprintf(stderr, "usage: process-lcmgl process-control-channel lcmgl-channel-name\n");
        fprintf(stderr, "    process-status-channel: LCM channel name with process control messages\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./process-lcmgl process_status\n");
}

void PrintStatus()
{
    mux.lock();
    printf("\rPlane time: %s\t\tLogfile: %s", stringsOut.time.c_str(), stringsOut.logfilesize.c_str());
    fflush(stdout);
    mux.unlock();
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_process_control_unsubscribe(lcm, process_sub);
    lcmt_log_size_unsubscribe(lcm, log_size_sub);
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

void process_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_process_control *msg, void *user)
{
    mux.lock();

    char tmbuf[64], buf[64];
    
    // figure out what time the plane thinks it is
    struct tm *nowtm;
    time_t tv_sec = msg->timestamp / 1000000.0;
    nowtm = localtime(&tv_sec);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
    sprintf(buf, "%s", tmbuf);


    stringsOut.time = buf;
    
    mux.unlock();
    
    
    PrintStatus();
}

void log_size_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_log_size *msg, void *user)
{

    mux.lock();
    
    char buf[500];
    
    // got a log size message, display it
    sprintf(buf, "#%d, %010d", msg->log_number, msg->log_size);
    stringsOut.logfilesize = buf;
    


    mux.unlock();
    
    
    PrintStatus();
}


int main(int argc,char** argv)
{
    stringsOut.time = "-------------------";
    stringsOut.logfilesize = "---";
    char *channelProcess = NULL;
    char *channelLogSize = NULL;
    
    if (argc!=3) {
        usage();
        exit(0);
    }

    channelProcess = argv[1];
    channelLogSize = argv[2];

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    

    signal(SIGINT,sighandler);
    
    
    process_sub = lcmt_process_control_subscribe(lcm, channelProcess, &process_handler, NULL);
    log_size_sub = lcmt_log_size_subscribe(lcm, channelLogSize, &log_size_handler, NULL);
    
    printf("Receiving:\n\t%s\n\t%s\n", channelProcess, channelLogSize);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
