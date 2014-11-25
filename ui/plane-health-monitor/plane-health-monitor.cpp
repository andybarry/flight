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
#include "../../LCM/lcmt_stereo_monitor.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>

#include "../../externals/ConciseArgs.hpp"

lcm_t * lcm;

mutex mux;


lcmt_process_control_subscription_t *process_sub;
lcmt_log_size_subscription_t *log_size_sub;
lcmt_stereo_monitor_subscription_t *stereo_monitor_sub;


struct StringsOutStruct {
    string time;
    string logfilesize;
    string frame_number;
};

StringsOutStruct stringsOut;




void PrintStatus()
{
    mux.lock();

    printf("\rTime: %s\tLogfile: %s        Frame #: %s", stringsOut.time.c_str(), stringsOut.logfilesize.c_str(), stringsOut.frame_number.c_str());

    fflush(stdout);
    mux.unlock();
}

void UpdateTimestamp(long timestamp)
{
     mux.lock();

    char tmbuf[64], buf[64];

    // figure out what time the plane thinks it is
    struct tm *nowtm;
    time_t tv_sec = timestamp / 1000000.0;
    nowtm = localtime(&tv_sec);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
    sprintf(buf, "%s", tmbuf);


    stringsOut.time = buf;

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
   UpdateTimestamp(msg->timestamp);
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

    UpdateTimestamp(msg->timestamp);


    PrintStatus();
}

void stereo_monitor_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_monitor *msg, void *user)
{

    mux.lock();

    char buf[500];

    // got a log size message, display it
    sprintf(buf, "%05d", msg->frame_number);
    stringsOut.frame_number = buf;



    mux.unlock();

    UpdateTimestamp(msg->timestamp);

    PrintStatus();
}


int main(int argc,char** argv)
{
    stringsOut.time = "-------------------";
    stringsOut.logfilesize = "---";
    string channel_process_str = "process_status";
    string channel_log_size_str = "log_size";
    string channel_stereo_monitor_str = "stereo_monitor";

    ConciseArgs parser(argc, argv);
    parser.add(channel_process_str, "p", "process-control-channel",
        "LCM channel for process control");
    parser.add(channel_log_size_str, "l", "log-size-channel",
        "LCM channel for log size");
    parser.add(channel_stereo_monitor_str, "s", "stereo-monitor",
        "LCM channel for stereo-monitor");
    parser.parse();

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }



    signal(SIGINT,sighandler);


    process_sub = lcmt_process_control_subscribe(lcm,
        channel_process_str.c_str(), &process_handler, NULL);
    log_size_sub = lcmt_log_size_subscribe(lcm,
        channel_log_size_str.c_str(), &log_size_handler, NULL);
    stereo_monitor_sub = lcmt_stereo_monitor_subscribe(lcm,
        channel_stereo_monitor_str.c_str(), &stereo_monitor_handler, NULL);


    printf("Receiving:\n\t%s\n\t%s\n\t%s\n--------------------------------------\n",
        channel_process_str.c_str(), channel_log_size_str.c_str(), channel_stereo_monitor_str.c_str());

    PrintStatus();

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
