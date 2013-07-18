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
#include <sys/stat.h>
#include <unistd.h>

#include <dirent.h>


#include "../../LCM/lcmt_log_size.h"

#include <bot_core/bot_core.h>
#include <bot_param/param_client.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
   
   

lcm_t * lcm;

char *channelLogSize = NULL;
string logDir;


static void usage(void)
{
        fprintf(stderr, "usage: log-monitor log-size-channel full-directory-path\n");
        fprintf(stderr, "    log-size-channel: LCM channel name with publish log size messages on\n");
        fprintf(stderr, "    full directory path: directory logfiles are\n");
        fprintf(stderr, "  example:\n");
        fprintf(stderr, "    ./log-monitor log_size /home/linaro/\n");
}



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

long GetFileSize(std::string filename)
{
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}


int main(int argc,char** argv)
{
    if (argc!=3) {
        usage();
        exit(0);
    }

    channelLogSize = argv[1];
    logDir = string(argv[2]);
    
    if (!logDir.empty())
    {
        char lastChar = *logDir.rbegin();
        if (lastChar != '/')
        {
            logDir = logDir + "/";
        }
    } else {
        fprintf(stderr, "directory can't be empty.  Try \".\" if you want this directory.\n");
        return 1;
    }
    

    lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    if (!lcm)
    {
        fprintf(stderr, "lcm_create failed.  Quitting.\n");
        return 1;
    }

    signal(SIGINT,sighandler);
    
    // get today's date now, so if it rolls over while we're still running it won't be an issue
    time_t rawtime;
    struct tm * timeinfo;
    char dateString [80];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    strftime (dateString,80,"%Y-%m-%d",timeinfo);
    puts (dateString);
    
    string lcmPrefix = "lcmlog-" + string(dateString) + ".";
    
    
    printf("Publishing: Log size:\n\t%s\n\n", channelLogSize);

    while (true)
    {
        // find the logfile for today
        // formatted something like "lcmlog-2013-07-18.00"
        
        int largestLog = -1;
        string theLog;
        
        DIR *dir;
        struct dirent *ent;
        if ((dir = opendir (logDir.c_str())) != NULL) {
          /* print all the files and directories within directory */
          while ((ent = readdir (dir)) != NULL) {
            //printf ("%s\n", ent->d_name);
            // see if this name starts with lcmlog-
            
            string thisName = ent->d_name;
            
            if (!thisName.compare(0, lcmPrefix.size(), lcmPrefix))
            {
                // get the number of this log
                string numString = thisName.substr(lcmPrefix.size());
                int logNum = atoi(numString.c_str());
                
                if (logNum > largestLog)
                {
                    largestLog = logNum;
                    theLog = thisName;
                }
            }
            
          }
          closedir (dir);
        } else {
          /* could not open directory */
          perror ("");
          return EXIT_FAILURE;
        }
        
        if (largestLog < 0)
        {
            // didn't find a log
            cout << "Failed to find any logs" << endl;
        } else {
            // found a log, get the filesize
            
            
            cout << "Filesize of log " << theLog << " is " << GetFileSize(string(logDir) + theLog) << endl;
            
            // publish to LCM
            lcmt_log_size msg;

            msg.timestamp = getTimestampNow();

            msg.log_number = largestLog;
            msg.log_size = GetFileSize(string(logDir) + theLog);

            lcmt_log_size_publish (lcm, channelLogSize, &msg);
            
        }
        
        sleep(1);
    }

    return 0;
}
