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

string fan_auto_manual_file = "/sys/devices/platform/odroidu2-fan/fan_mode";

string fan_pwm_file = "/sys/devices/platform/odroidu2-fan/pwm_duty";


lcm_t * lcm;

bool disable_fan_control = false;

string cpu_info_channel_str = "cpu-info-hostname";


void sighandler(int dum) {
    printf("\nRestoring automatic fan control...");
    ofstream auto_man_file;
    auto_man_file.open(fan_auto_manual_file);
    auto_man_file << "auto";
    auto_man_file.close();

    printf("\nClosing... ");
    lcm_destroy (lcm);

    printf("done.\n");


    exit(0);
}

void SetFanSpeed(float cpu_temp, int fan_pwm) {

    int new_pwm;

    if (cpu_temp >= 70) {
        new_pwm = 255;
    } else {

        if (cpu_temp >= 65) {

            if (fan_pwm == 128) {
                new_pwm = 128;
            } else {
                new_pwm = 255;
            }
        } else {

            if (cpu_temp >= 60) {

                if (fan_pwm < 5) {

                    new_pwm = fan_pwm;

                } else {
                    new_pwm = 128;
                }
            } else {

                if (fan_pwm > 5) { // 0 doesn't report 0, it reports a small number
                    new_pwm = 0;
                } else {
                    new_pwm = fan_pwm;
                }
            }

        }
    }

    if (new_pwm != fan_pwm) {
        // set the new fan speed

        ofstream pwm_file;
        pwm_file.open(fan_pwm_file);
        pwm_file << std::to_string(new_pwm);
        pwm_file.close();

    }
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

    // get CPU temp
    float cpu_temp = stof(cpu_temp_string);
    cpu_temp /= 1000;
    msg.cpu_temp = cpu_temp;


    // get fan speed
    if (!disable_fan_control) {
        ifstream fan_file_stream(fan_pwm_file);
        string fan_string;
        getline(fan_file_stream, fan_string);
        fan_file_stream.close();

        fan_string = fan_string.substr(22, string::npos);
        std::size_t first_space = fan_string.find_first_of(" ");

        fan_string = fan_string.substr(0, first_space);

        int fan_pwm = stoi(fan_string);
        msg.fan_pwm = fan_pwm;

        SetFanSpeed(cpu_temp, fan_pwm);
    } else {
        msg.fan_pwm = -1;
    }

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
    parser.add(disable_fan_control, "n", "disable-fan-control", "Pass to disable ODROID-U3 fan control.");
    parser.parse();

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm) {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    signal(SIGINT,sighandler);

    if (!disable_fan_control) {
        printf("Taking fan control...\n");

        ofstream auto_man_file;
        auto_man_file.open(fan_auto_manual_file);
        auto_man_file << "manual";
        auto_man_file.close();
    }

    printf("Publishing:\n\tCPU Info: %s\n", cpu_info_channel_str.c_str());

    while (true) {

        PublishCpuInfo();

        // wait 1 second
        sleep(1);
    }

    return 0;
}
