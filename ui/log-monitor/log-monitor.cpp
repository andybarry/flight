/*
 * Monitors the log file size and sends LCM messages about it
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
#include <sys/types.h>

#include "../../LCM/lcmt_log_size.h"

#include "../../externals/ConciseArgs.hpp"

#include "../../utils/utils/RealtimeUtils.hpp"

#include <sys/statvfs.h>

lcm_t * lcm;

string log_dir = "/home/odroid";
string log_info_channel_str = "log-info-hostname";


void sighandler(int dum) {
    printf("\nClosing... ");

    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}

long GetFileSize(std::string filename) {
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}


int main(int argc,char** argv) {

    // use this computers hostname by default
    char hostname[100];
    size_t hostname_len = 100;

    gethostname(hostname, hostname_len);
    log_info_channel_str = "log-info-" + string(hostname);

    ConciseArgs parser(argc, argv);
    parser.add(log_info_channel_str, "c", "log-info-channel",
        "LCM channel for publishing log info.");
    parser.add(log_dir, "d", "log-directory",
        "Directory containing log files.", true);
    parser.parse();

    if (!log_dir.empty()) {
        char last_char = *log_dir.rbegin();
        if (last_char != '/') {
            log_dir = log_dir + "/";
        }
    } else {
        fprintf(stderr, "directory can't be empty.  Try \".\" if you want this directory.\n");
        return 1;
    }


    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm) {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    signal(SIGINT,sighandler);



    printf("Publishing:\n\tLog size: %s\n\n", log_info_channel_str.c_str());

    while (true) {
        // find the logfile for today
        // formatted something like "lcmlog-2013-07-18.00"

        // get the date
        time_t rawtime;
        struct tm * timeinfo;
        char date_string[80];

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );

        strftime (date_string, 80, "%Y-%m-%d", timeinfo);

        string lcm_prefix = "lcmlog-" + string(date_string) + ".";

        int largest_log = -1;
        int log_size = -1;
        string the_log;

        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (log_dir.c_str())) != NULL) {
            /* print all the files and directories within directory */
            while ((ent = readdir (dir)) != NULL) {
                //printf ("%s\n", ent->d_name);
                // see if this name starts with lcmlog-

                string this_name = ent->d_name;

                if (!this_name.compare(0, lcm_prefix.size(), lcm_prefix)) {
                    // get the number of this log
                    string num_string = this_name.substr(lcm_prefix.size());
                    int log_num = atoi(num_string.c_str());

                    if (log_num > largest_log) {
                        largest_log = log_num;
                        the_log = this_name;
                    }
                }

            }
            closedir (dir);
        } else {
          /* could not open directory */
          perror ("");
          return EXIT_FAILURE;
        }

        if (largest_log < 0) {
            // didn't find a log
            //cout << "Failed to find any logs" << endl;

            log_size = -1;

        } else {
            // found a log, get the filesize


            //cout << "Filesize of log " << theLog << " is " << GetFileSize(string(logDir) + theLog) << endl;

            log_size = GetFileSize(string(log_dir) + the_log);

        }

        // get disk space free
        struct statvfs fi_data;
        double disk_free = -1;

        if((statvfs(log_dir.c_str(), &fi_data)) < 0 ) {
            printf("Failed to stat %s:\n", log_dir.c_str());
        } else {
            disk_free = (double)fi_data.f_bavail * (double)fi_data.f_bsize / 1048576.0d;
        }

        // publish to LCM
        lcmt_log_size msg;

        msg.timestamp = GetTimestampNow();

        msg.log_number = largest_log;
        msg.log_size = log_size;

        msg.disk_space_free = disk_free;

        lcmt_log_size_publish (lcm, log_info_channel_str.c_str(), &msg);

        sleep(1);
    }

    return 0;
}
