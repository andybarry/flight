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

std::string video_directory;

lcmt_stereo_subscription_t *stereo_sub;

cv::VideoCapture video_capture;

int current_video_number = -1;


void mono_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo *msg, void *user) {
    // open the frame of the video specified by the message

    // check to see if the current video is the correct video file

    if (current_video_number != msg->video_number) {

        video_capture.release();

        std::string newfile = GetMonoFilename(msg->video_number);

        std::cout << "Opening file: " << newfile << std::endl;

        if (!video_capture.open(newfile)) {
            std::cerr << "Failed to open file: " << newfile << std::endl;
        }
        current_video_number = msg->video_number;
    }

    video_capture.set(CV_CAP_PROP_POS_FRAMES, msg->frame_number);

    cv::Mat frame;

    video_capture >> frame;

    SendImageOverLcm(lcm_, image_channel, frame, -1);

}


std::string GetMonoFilename(int number) {

    // TODO: extract date from directory

    boost::format formatter = boost::format("mono-2015-05-01.%02d.avi") % number;
    return video_directory + formatter.str();
}


int main(int argc,char** argv) {

    std::string mono_channel = "stereo-mono";


    ConciseArgs parser(argc, argv);
    parser.add(mono_channel, "m", "stereo-mono-channel", "LCM channel to receive stereo-mono messages on.");
    parser.add(image_channel, "i", "image-channel", "LCM channel to publish images on to.");
    parser.add(video_directory, "d", "directory", "Directory to read video files from", true);
    parser.parse();


    lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm_) {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    // control-c handler
    signal(SIGINT,sighandler);

    // check to make sure the directory exists:

    // TODO

    stereo_sub = lcmt_stereo_subscribe(lcm_, mono_channel.c_str(), &mono_handler, NULL);

    printf("LCM:\n\t%s\n\t%s\n", mono_channel.c_str(), image_channel.c_str());


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
