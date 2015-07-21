/*
 * Ultra-lightweight process control
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
#include <glib.h> // for configuration files

#include "../../LCM/lcmt_process_control.h"
#include "../../LCM/lcmt_stereo_control.h"

#include "ProcessControlProc.hpp"
#include <map>

lcm_t * lcm;

char *channelStereoControl = NULL;
char *channelProcessReport = NULL;

lcmt_process_control_subscription_t *process_control_sub;

std::map<std::string, ProcessControlProc> processMap;

static void usage(void)
{
        fprintf(stderr, "usage: process-control chan-process-control chan-stereo-control chan-process-report config-file\n");
        fprintf(stderr, "    chan-process-control: LCM channel with process_control messages\n");
        fprintf(stderr, "    chan-stereo-control: TODO\n");
        fprintf(stderr, "    chan-process report: publishes process reports on this channel\n");
        fprintf(stderr, "    configfile: config file listing processes and arguments\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./process-control process_control stereo_control process_status ../../config/processControl.conf\n");
        fprintf(stderr, "    reads LCM process-control messages and starts/stops processes based on those commands.\n");
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
    //processMap.at("paramServer").PrintIO();
    if (msg->paramServer == 1)
    {
        processMap.at("paramServer").StartProcess();
    } else if (msg->paramServer == 2) {
        processMap.at("paramServer").StopProcess();
    }

    if (msg->mavlinkLcmBridge == 1)
    {
        processMap.at("mavlinkLcmBridge").StartProcess();
    } else if (msg->mavlinkLcmBridge == 2) {
        processMap.at("mavlinkLcmBridge").StopProcess();
    }

    if (msg->mavlinkSerial == 1)
    {
        processMap.at("mavlinkSerial").StartProcess();
    } else if (msg->mavlinkSerial == 2) {
        processMap.at("mavlinkSerial").StopProcess();
    }

    if (msg->stateEstimator == 1)
    {
        processMap.at("stateEstimator").StartProcess();
    } else if (msg->stateEstimator == 2) {
        processMap.at("stateEstimator").StopProcess();
    }

    if (msg->windEstimator == 1)
    {
        processMap.at("windEstimator").StartProcess();
    } else if (msg->windEstimator == 2) {
        processMap.at("windEstimator").StopProcess();
    }

    if (msg->controller == 1)
    {
        processMap.at("controller").StartProcess();
    } else if (msg->controller == 2) {
        processMap.at("controller").StopProcess();
    }

    if (msg->logger == 1)
    {
        processMap.at("logger").StartProcess();
    } else if (msg->logger == 2) {
        processMap.at("logger").StopProcess();
    }

    if (msg->stereo == 1)
    {
        processMap.at("stereo").StartProcess();
    } else if (msg->stereo == 2) {
        processMap.at("stereo").StopProcess();
    }
}

void CheckForProc(std::string procString)
{
    if (processMap.find(procString) == processMap.end())
    {
        fprintf(stderr, "Error: configuration file does not specifiy group for %s\n", procString.c_str());
        exit(-1);
    }
}

// thread that sends a status message every n seconds
// threaded lcm reading
void* ProcessStatusThreadFunc(void *nothing)
{
    while (true)
    {
        // sleep for 1 second
        sleep(1);
        // send process status messages


        // get a new lcm message
        lcmt_process_control statMsg;

        statMsg.timestamp = getTimestampNow();

        statMsg.paramServer = processMap.at("paramServer").IsAlive();
        statMsg.mavlinkLcmBridge = processMap.at("mavlinkLcmBridge").IsAlive();
        statMsg.mavlinkSerial = processMap.at("mavlinkSerial").IsAlive();
        statMsg.stateEstimator = processMap.at("stateEstimator").IsAlive();
        statMsg.windEstimator = processMap.at("windEstimator").IsAlive();
        statMsg.controller = processMap.at("controller").IsAlive();
        statMsg.logger = processMap.at("logger").IsAlive();
        statMsg.stereo = processMap.at("stereo").IsAlive();


        // send the message
        lcmt_process_control_publish (lcm, channelProcessReport, &statMsg);


    }
    return NULL;
}

int main(int argc,char** argv)
{

    char *channelProcessControl = NULL;
    char *configurationFile = NULL;

    if (argc!=5) {
        usage();
        exit(0);
    }

    channelProcessControl = argv[1];
    channelStereoControl = argv[2];
    channelProcessReport = argv[3];
    configurationFile = argv[4];


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
            std::cout << "Error: no arguments list for process " << processGroups[i] << std::endl;
        } else {
            processMap.insert(std::pair<std::string, ProcessControlProc>(processGroups[i], ProcessControlProc(thisArgs, (int)length)));
        }
    }

    // now throw an error if the configuration file doesn't have all the right parts
    CheckForProc("paramServer");
    CheckForProc("mavlinkLcmBridge");
    CheckForProc("mavlinkSerial");
    CheckForProc("stateEstimator");
    CheckForProc("windEstimator");
    CheckForProc("controller");
    CheckForProc("logger");
    CheckForProc("stereo");


    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    process_control_sub = lcmt_process_control_subscribe(lcm, channelProcessControl, &procces_control_handler, NULL);

    signal(SIGINT,sighandler);

    pthread_t processStatusThread;

    pthread_create( &processStatusThread, NULL, ProcessStatusThreadFunc, NULL);

    printf("Receiving:\n\tProcess Control LCM: %s\nPublishing LCM:\n\tStereo: %s\n\tStatus: %s\n", channelProcessControl, channelStereoControl, channelProcessReport);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}
