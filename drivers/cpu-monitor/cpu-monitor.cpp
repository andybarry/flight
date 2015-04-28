/*
 * Monitors the CPU clock and temperature and publishes info to LCM.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <dirent.h>

#include <fstream>


#include "../../LCM/lcmt_cpu_info.h"

#include "../../externals/ConciseArgs.hpp"

#include "../../utils/utils/RealtimeUtils.hpp"


string cpu_freq_file =  "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq";

string cpu_temp_file = "/sys/class/thermal/thermal_zone0/temp";


lcm_t * lcm;

string cpu_info_channel_str = "cpu-info-hostname";


void sighandler(int dum)
{
    printf("\nClosing... ");
    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}

void PublishCpuInfo() {

    // get cpu freq

    ifstream cpu_freq_file_stream(cpu_freq_file);
    string cpu_freq_string;
    getline(cpu_freq_file_stream, cpu_freq_string);



    ifstream cpu_temp_file_stream(cpu_temp_file);
    string cpu_temp_string;
    getline(cpu_temp_file_stream, cpu_temp_string);


    int cpu_freq = stoi(cpu_freq_string);

    lcmt_cpu_info msg;
    msg.cpu_freq = cpu_freq;

    float cpu_temp = stof(cpu_temp_string);
    cpu_temp /= 1000;
    msg.cpu_temp = cpu_temp;

    msg.timestamp = GetTimestampNow();

    lcmt_cpu_info_publish(lcm, cpu_info_channel_str.c_str(), &msg);


}

int main(int argc,char** argv) {

    // use this computers hostname by default
    char hostname[100];
    size_t hostname_len = 100;

    gethostname(hostname, hostname_len);
    cpu_info_channel_str = "cpu-info-" + string(hostname);

    ConciseArgs parser(argc, argv);
    parser.add(cpu_info_channel_str, "c", "cpu-info-channel",
        "LCM channel for publishing CPU info.");
    parser.add(cpu_freq_file, "f", "cpu-freq-file",
        "File to read containing CPU frequency.");
    parser.add(cpu_temp_file, "t", "cpu-temp-file",
        "File to read containing CPU temperature.");
    parser.parse();

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm) {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    signal(SIGINT,sighandler);

    printf("Publishing:\n\tCPU Info: %s\n", cpu_info_channel_str.c_str());

    while (true) {

        PublishCpuInfo();

        // wait 1 second
        sleep(1);
    }

    return 0;
}
