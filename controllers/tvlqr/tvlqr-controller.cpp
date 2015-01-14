/*
 * Implements a Time Varying Linear Quadratic Regulator using trajectories.
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2014
 *
 */

#include "tvlqr-controller.hpp"

using namespace std;

lcm_t * lcm;
char *lcm_out = NULL;
int numFrames = 0;
unsigned long totalTime = 0;

// global trajectory library
TrajectoryLibrary trajlib;

bot_lcmgl_t* lcmgl;


int main(int argc,char** argv) {

    bool ttl_one = false;
    string config_file = "";
    string trajectory_dir = "";

    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(disable_filtering, "f", "disable-filtering", "Disable filtering.");
    parser.add(config_file, "c", "config", "Configuration file containing camera GUIDs, etc.", true);
    parser.add(trajectory_dir, "d", "trajectory-dir" "Directory containing CSV files with trajectories.");
    parser.parse();

    if (disable_filtering) {
        cout << "WARNING: you have disabled filtering with the -f flag." << endl;
    }

    OpenCvStereoConfig stereo_config;

    // parse the config file
    if (ParseConfigFile(config_file, &stereo_config) != true)
    {
        fprintf(stderr, "Failed to parse configuration file, quitting.\n");
        return 1;
    }

    // load calibration
    OpenCvStereoCalibration stereo_calibration;

    if (LoadCalibration(stereo_config.calibrationDir, &stereo_calibration) != true)
    {
        cerr << "Error: failed to read calibration files. Quitting." << endl;
        return 1;
    }

    if (trajectory_dir != "") {
        // load a trajectory library
        if (!trajlib.LoadLibrary(trajectory_dir)) {
            cerr << "Error: failed to load trajectory library.  Quitting." << endl;
            return 1;
        }

        //trajlib.Print();
    }


    if (ttl_one) {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=1");
    } else {
        lcm = lcm_create ("udpm://239.255.76.67:7667?ttl=0");
    }

    if (!lcm)
    {
        fprintf(stderr, "lcm_create for recieve failed.  Quitting.\n");
        return 1;
    }

    // init frames
    BotParam *param = bot_param_new_from_server(lcm, 0);
    BotFrames *bot_frames = bot_frames_new(lcm, param);

    // init octomap
    StereoOctomap octomap(bot_frames);

    octomap.SetStereoConfig(stereo_config, stereo_calibration);

    StereoFilter filter(0.1);

    StereoHandlerData user_data;
    user_data.octomap = &octomap;
    user_data.filter = &filter;

    char *stereo_channel;
    if (bot_param_get_str(param, "lcm_channels.stereo", &stereo_channel) >= 0) {
        stereo_sub = lcmt_stereo_subscribe(lcm, stereo_channel, &stereo_handler, &user_data);
    }

    lcmgl = bot_lcmgl_init(lcm, "lcmgl-stereo-transformed");
    bot_lcmgl_enable(lcmgl, GL_BLEND);




    // control-c handler
    signal(SIGINT,sighandler);

    printf("Receiving LCM:\n\tStereo: %s\n", stereo_channel);

    while (true)
    {
        // read the LCM channel
        lcm_handle (lcm);
    }

    return 0;
}

void sighandler(int dum)
{
    printf("\n\nclosing... ");

    lcmt_stereo_unsubscribe(lcm, stereo_sub);
    lcm_destroy (lcm);

    printf("done.\n");

    exit(0);
}
