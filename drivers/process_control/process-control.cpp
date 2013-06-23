/*
 * Ultra-lightweight process control
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
#include <glib.h> // for configuration files

#include "../../LCM/lcmt_process_control.h"
#include "../../LCM/lcmt_stereo_control.h"

#include "mavconn.h" // from mavconn
    
#include "../../mavlink-generated2/csailrlg/mavlink.h"

#include "ProcessControlProc.hpp"
#include <map>

lcm_t * lcm;

char *channelStereoControl = NULL;

lcmt_process_control_subscription_t *process_control_sub;

std::map<string, ProcessControlProc> processMap;

uint8_t systemID = getSystemID();

static void usage(void)
{
        fprintf(stderr, "usage: TODOardupilot-mavlink-bridge mavlink-channel-name attitude-channel-name baro-airspeed-channel-name gps-channel-name battery-status-channel-name input-servo-channel-name output-servo-channel-name\n");
        fprintf(stderr, "    mavlink-channel-name : LCM channel name with MAVLINK LCM messages\n");
        fprintf(stderr, "    attitude-channel-name : LCM channel to publish attitude messages on\n");
        fprintf(stderr, "    baro-airspeed-channel-name : LCM channel to publish barometric altitude and airspeed\n");
        fprintf(stderr, "    gps-channel-name : LCM channel to publish GPS messages on\n");
        fprintf(stderr, "    battery-status-channel-name : LCM channel to publish battery status messages on\n");
        fprintf(stderr, "    input-servo-channel-name : LCM channel to listen for servo commands on\n");
        fprintf(stderr, "    output-servo-channel-name : LCM channel to publish executed servo commands on\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./ardupilot-mavlink-bridge MAVLINK attitude baro-airspeed gps battery-status wingeron_u servo_out\n");
        fprintf(stderr, "    reads LCM MAVLINK messages and converts them to easy to use attitude, baro/airspeed and gps messages.  Also pushes servo commands to the APM and reads the executed commands.\n");
}


void sighandler(int dum)
{
    printf("\nClosing... ");

    lcmt_process_control_unsubscribe(lcm, process_control_sub);
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

void procces_control_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_process_control *msg, void *user)
{
    // got a process control message
    
    // start or stop processes based on it
    
    // the message sends the full requested state at once
    
    if (msg->paramServer == true)
    {
        processMap.at("paramServer").StartProcess();
    } else {
        processMap.at("paramServer").StopProcess();
    }
    
    if (msg->mavlinkLcmBridge == true)
    {
        processMap.at("mavlinkLcmBridge").StartProcess();
    } else {
        processMap.at("mavlinkLcmBridge").StopProcess();
    }
    
    if (msg->mavlinkSerial == true)
    {
        processMap.at("mavlinkSerial").StartProcess();
    } else {
        processMap.at("mavlinkSerial").StopProcess();
    }
    
    if (msg->stateEstimator == true)
    {
        processMap.at("stateEstimator").StartProcess();
    } else {
        processMap.at("stateEstimator").StopProcess();
    }
    
    if (msg->windEstimator == true)
    {
        processMap.at("windEstimator").StartProcess();
    } else {
        processMap.at("windEstimator").StopProcess();
    }
    
    if (msg->controller == true)
    {
        processMap.at("controller").StartProcess();
    } else {
        processMap.at("controller").StopProcess();
    }
    
    if (msg->logger == true)
    {
        processMap.at("logger").StartProcess();
    } else {
        processMap.at("logger").StopProcess();
    }
    
    if (msg->stereo == true)
    {
        // TODO STEREO
    } else {
        // TODO STEREO
    }
    
    
}



int main(int argc,char** argv)
{
    
    char *channelProcessControl = NULL;
    char *configurationFile = NULL;

    if (argc!=4) {
        usage();
        exit(0);
    }
    
    channelProcessControl = argv[1];
    channelStereoControl = argv[2];
    configurationFile = argv[3];


    // read the configuration file to get the processes we'll need
    GKeyFile *keyfile;
    GKeyFileFlags flags = G_KEY_FILE_NONE;
    GError *error = NULL;
    gsize length, length2;

    /* Create a new GKeyFile object and a bitwise list of flags. */
    keyfile = g_key_file_new ();

    /* Load the GKeyFile from keyfile.conf or return. */
    if (!g_key_file_load_from_file (keyfile, configurationFile, flags, &error))
    {
        fprintf(stderr, "Configuration file \"%s\" not found.\n", configurationFile);
        return -1;
    }
    
    // build the process table based on the configuration file
    
    // first get the names of all of the processes
    char **processGroups = g_key_file_get_groups(keyfile, &length);
    
    for (int i=0; i<(int)length; i++)
    {
        // for each process...
        
        // get the arguments
        char **thisArgs = g_key_file_get_string_list(keyfile, processGroups[i], "arguments", &length2, &error);
        
        if (thisArgs == NULL)
        {
            cout << "Error: no arguments list for process " << processGroups[i] << endl;
        } else {
            processMap.insert(std::pair<string, ProcessControlProc>(processGroups[i], ProcessControlProc(thisArgs, (int)length)));
        }
    }
    

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    process_control_sub = lcmt_process_control_subscribe(lcm, channelProcessControl, &procces_control_handler, NULL);

    signal(SIGINT,sighandler);
    
    printf("Receiving:\n\tProcess Control LCM: %s\nPublishing LCM:\n\tStereo: %s\n", channelProcessControl, channelStereoControl);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
