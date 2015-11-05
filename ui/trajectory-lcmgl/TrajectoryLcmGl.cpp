#include "TrajectoryLcmGl.hpp"
#include "../../externals/ConciseArgs.hpp"

TrajectoryLcmGl::TrajectoryLcmGl(lcm::LCM *lcm, const TrajectoryLibrary *trajlib) {
    lcm_ = lcm;
    trajlib_ = trajlib;
    BotParam *param = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    bot_frames_ = bot_frames_new(lcm_->getUnderlyingLCM(), param);
}

void TrajectoryLcmGl::ProcessTrajectoryMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::tvlqr_controller_action *msg) {
    DrawTrajectoryLcmGl(msg->trajectory_number, msg->timestamp);
}

void TrajectoryLcmGl::DrawTrajectoryLcmGl(int traj_number, int64_t timestamp) {
    std::cout << "Drawing t=" << timestamp << std::endl;

    if (!last_traj_name_.empty() && timestamp > last_traj_timestamp_) {
        // redraw the old trajectory to remove anything that we didn't actually execute
        bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm_->getUnderlyingLCM(), last_traj_name_.c_str());
        bot_lcmgl_color3f(lcmgl, 0, 0, 1);

        last_traj_->Draw(lcmgl, &last_draw_transform_, ConvertTimestampToSeconds(timestamp - last_traj_timestamp_));
        bot_lcmgl_switch_buffer(lcmgl);
        bot_lcmgl_destroy(lcmgl);
    }

    // draw the trajectory via lcmgl
    std::string name = "Trajectory playback t=" + std::to_string(timestamp);
    BotTrans body_to_local;
    bot_frames_get_trans(bot_frames_, "body", "local", &body_to_local);

    bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm_->getUnderlyingLCM(), name.c_str());
    bot_lcmgl_color3f(lcmgl, 0, 0, 1);

    const Trajectory *traj = trajlib_->GetTrajectoryByNumber(traj_number);
    traj->Draw(lcmgl, &body_to_local);
    bot_lcmgl_switch_buffer(lcmgl);
    bot_lcmgl_destroy(lcmgl);

    last_traj_name_ = name;
    last_draw_transform_ = body_to_local;
    last_traj_timestamp_ = timestamp;
    last_traj_ = traj;
    std::cout << "done" << std::endl;
}

int main(int argc,char** argv) {

    std::string tvlqr_action_channel = "tvlqr-action";


    ConciseArgs parser(argc, argv);
    parser.add(tvlqr_action_channel, "t", "tvlqr-action", "LCM channel to receive which TVLQR trajectory is running on.");

    parser.parse();


    std::string lcm_url;
    // create an lcm instance
    lcm_url = "udpm://239.255.76.67:7667?ttl=0";
    lcm::LCM lcm(lcm_url);

    if (!lcm.good()) {
        std::cerr << "LCM creation failed." << std::endl;
        return 1;
    }

    BotParam *param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 0);


    std::string trajectory_dir = std::string(bot_param_get_str_or_fail(param, "tvlqr_controller.library_dir"));
    trajectory_dir = ReplaceUserVarInPath(trajectory_dir);

    TrajectoryLibrary trajlib;
    trajlib.LoadLibrary(trajectory_dir);

    TrajectoryLcmGl traj_lcmgl(&lcm, &trajlib);

    // subscribe to LCM channels
    lcm.subscribe(tvlqr_action_channel, &TrajectoryLcmGl::ProcessTrajectoryMsg, &traj_lcmgl);

    printf("Receiving LCM:\n\tTrajectories: %s\n", tvlqr_action_channel.c_str());

    while(0 == lcm.handle());

    return 0;
}
