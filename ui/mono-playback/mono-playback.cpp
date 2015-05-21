/*
 * Reads LCM messages and a mono video file and publishes
 * the images to LCM for viewing in the HUD.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "mono-playback.hpp"
#include "../../externals/ConciseArgs.hpp"
#include "opencv2/opencv.hpp"

lcm_t * lcm_;

std::string image_channel = "stereo_image_left";

std::string video_directory = "";
std::string log_directory = "";

lcmt_stereo_subscription_t *stereo_sub;

cv::VideoCapture video_capture;

int current_video_number = -1;

int frame_offset = 0;


void mono_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    // open the frame of the video specified by the message

    // check to see if the current video is the correct video file

    if (current_video_number != msg->video_number) {

        video_capture.release();

        std::string newfile = GetMonoFilename(msg->timestamp, msg->video_number);

        std::cout << "Opening file: " << newfile << std::endl;

        if (!video_capture.open(newfile)) {
            std::cerr << "Failed to open file: " << newfile << std::endl;
            return;
        }
        current_video_number = msg->video_number;
    }

    video_capture.set(CV_CAP_PROP_POS_FRAMES, msg->frame_number + frame_offset);

    cv::Mat frame;

    video_capture >> frame;

    SendImageOverLcm(lcm_, image_channel, frame);

}


std::string GetMonoFilename(long timestamp, int video_number) {

    char tmbuf[64], buf[64];

    // figure out what time the plane thinks it is
    struct tm *nowtm;
    time_t tv_sec = timestamp / 1000000.0;
    nowtm = localtime(&tv_sec);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d", nowtm);
    sprintf(buf, "%s", tmbuf);

    std::string date_str = std::string(buf);

    std::string dir;

    if (video_directory != "") {
        dir = video_directory;
    } else {
        dir = log_directory + GetVideoDirectory(date_str);
    }

    if (dir == "") {
        std::cerr << "Warning: unable to find a video file.  You might consider using the -v option instead of -d." << std::endl;
        return "";
    }


    boost::format formatter = boost::format(".%02d.avi") % video_number;

    std::string path = dir + boost::filesystem::path::preferred_separator +
        "onboard-vids" + boost::filesystem::path::preferred_separator +
        + "mono-" + date_str + formatter.str();

    return path;
}

/**
 * Converts a timestamp into a date, searches for that date in the directory
 * given by the user, and returns the directory name if it is unique.  Returns the
 * first if there is more than one directory.
 */
std::string GetVideoDirectory(std::string date_str) {

    boost::filesystem::directory_iterator end_itr; // default construction
                                                   // yields past-the-end
    for (boost::filesystem::directory_iterator itr(log_directory);
        itr != end_itr; ++itr ) {

        // iterate through the directories availible


        string this_file = itr->path().leaf().string();

        if (this_file.find(date_str) != string::npos) {
            return this_file;
        }

    }

    return "";

}

int main(int argc,char** argv) {

    std::string mono_channel = "stereo-mono";


    ConciseArgs parser(argc, argv);
    parser.add(log_directory, "d", "log-directory", "Directory to read video files from");
    parser.add(video_directory, "v", "video-directory", "Use this to force a video directory path in case the automatic detection via the more general log-directory option does not work for you.");
    parser.add(mono_channel, "m", "stereo-mono-channel", "LCM channel to receive stereo-mono messages on.");
    parser.add(image_channel, "i", "image-channel", "LCM channel to publish images on to.");
    parser.add(frame_offset, "o", "frame-offset", "Offset frames this amount (to correct for sync issues.");
    parser.parse();

    if (log_directory == "" && video_directory == "") {
        std::cerr << "You must specify either a log-directory or a video-directory." << std::endl;
        return 1;
    }

    lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm_) {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }


    // control-c handler
    signal(SIGINT,sighandler);

    std::string dir;

    // ensure the directory has a trailing slash
    if (log_directory != "") {
        if (log_directory.at(log_directory.length() - 1) != boost::filesystem::path::preferred_separator) {
            log_directory = log_directory + boost::filesystem::path::preferred_separator;
        }

        if (!boost::filesystem::exists(log_directory)) {
            std::cerr << "Error: failed to find directory for video files: " << log_directory << std::endl;

            return false;
        }
        dir = log_directory;
    }

    if (video_directory != "") {
        if (video_directory.at(video_directory.length() - 1) != boost::filesystem::path::preferred_separator) {
            video_directory = video_directory + boost::filesystem::path::preferred_separator;
        }

        if (!boost::filesystem::exists(video_directory)) {
            std::cerr << "Error: failed to find directory for video files: " << video_directory << std::endl;

            return false;
        }
        dir = video_directory;
    }



    // check to make sure the directory exists:




    stereo_sub = lcmt_stereo_subscribe(lcm_, mono_channel.c_str(), &mono_handler, NULL);

    printf("LCM:\n\t%s\n\t%s\nVideo directory: %s\n", mono_channel.c_str(), image_channel.c_str(), dir.c_str());

    if (frame_offset != 0) {
        std::cerr << "WARNING: frame offset: " << frame_offset << std::endl;
    }


    while (true) {
        // read the LCM channel
        lcm_handle(lcm_);
    }

    return 0;

}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcm_destroy (lcm_);

    printf("done.\n");

    exit(0);
}
