/*
 * Reads an MP4 file from the GoPro and plays it back into a format that
 * the HUD can read, synced to LCM message playback.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include "video-playback.hpp"
#include "../../externals/ConciseArgs.hpp"
#include <highgui.h>
#include "opencv2/opencv.hpp"

lcm_t * lcm_;

std::string video_channel = "ground-video";
std::string servo_out_channel = "servo_out";

lcmt_deltawing_u_subscription_t *servo_out_sub;


int nth_frame = 0;
long nth_frame_time;

double at_frame = 0;
int fps = 120;

void servo_out_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_deltawing_u *msg, void *user) {

    double delta_t = (msg->timestamp - nth_frame_time) / 1000000.0;

    //at_frame = delta_t * fps - nth_frame;
    at_frame = delta_t * 1000;

    if (at_frame < 0) {
        at_frame = 0;
    }

    std::cout << "delta: " << delta_t << " at_frame: " << at_frame << std::endl;
}



int main(int argc,char** argv) {

    std::string filename;

    ConciseArgs parser(argc, argv);
    parser.add(video_channel, "v", "video-channel", "LCM channel to send video messages on.");
    parser.add(servo_out_channel, "s", "servo-out-channel", "LCM channel to receive servo messages on (for sync).");
    parser.add(filename, "f", "file", "Video file to play", true);
    parser.add(nth_frame, "n", "nth-frame", "Frame to record timestamp of.");
    parser.add(nth_frame_time, "t", "nth-frame-time", "Timestamp of the Nth frame", true);
    parser.add(fps, "p", "fps", "FPS of the video");
    parser.parse();


    lcm_ = lcm_create ("udpm://239.255.76.67:7667?ttl=0");

    if (!lcm_) {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    servo_out_sub = lcmt_deltawing_u_subscribe(lcm_, servo_out_channel.c_str(), &servo_out_handler, NULL);

    // control-c handler
    signal(SIGINT,sighandler);

    // open the MP4 file

    cvNamedWindow("Ground Video", CV_WINDOW_AUTOSIZE);

    cv::VideoCapture capture(filename);

    cv::Mat frame;

    printf("LCM:\n\t%s\n", video_channel.c_str());

    capture.set(CV_CAP_PROP_FPS, 5000);

    while (true) {
        // read the LCM channel
        NonBlockingLcm(lcm_);

        capture.set(CV_CAP_PROP_POS_MSEC, at_frame);

        //frameIndex += 4;
        /* grab frame image, and retrieve */
        capture >> frame;

        imshow("Ground Video", frame);
        /* if ESC is pressed then exit loop */
        char c = cvWaitKey(1);
        std::cout << "." << std::endl;
    }

    return 0;

}


void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcm_destroy (lcm_);

    printf("done.\n");

    cvDestroyWindow("Ground Video");

    exit(0);
}
