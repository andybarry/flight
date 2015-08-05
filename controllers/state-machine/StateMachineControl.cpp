#include "StateMachineControl.hpp"
#include "../../externals/ConciseArgs.hpp"

StateMachineControl::StateMachineControl(lcm::LCM *lcm, std::string traj_dir, BotFrames *bot_frames, double dist_threshold, int start_traj_num, std::string tvlqr_action_out_channel) : fsm_(*this) {
    lcm_ = lcm;
    bot_frames_ = bot_frames;

    octomap_ = new StereoOctomap(bot_frames_);

    trajlib_ = new TrajectoryLibrary();

    if (trajlib_->LoadLibrary(traj_dir) == false) {
        std::cerr << "ERROR: Failed to load trajectory library." << std::endl;
        exit(1);
    }

    current_traj_ = trajlib_->GetTrajectoryByNumber(start_traj_num);

    safe_distance_ = dist_threshold;

    tvlqr_action_out_channel_ = tvlqr_action_out_channel;
}

StateMachineControl::~StateMachineControl() {
    delete octomap_;
    delete trajlib_;
}

void StateMachineControl::ProcessImuMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const mav::pose_t *msg) {
    fsm_.ImuUpdate(*msg);
}

void StateMachineControl::ProcessStereoMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::stereo *msg) {
    octomap_->ProcessStereoMessage(msg);
}


bool StateMachineControl::CheckForObstacles() {
    // search for an obstacle in the path

    BotTrans body_to_local;
    bot_frames_get_trans(bot_frames_, "body", "local", &body_to_local);

    double t;

    if (current_traj_->IsTimeInvariant()) {
        t = 0;
    } else if (traj_start_t_ > 0) {
        t = (GetTimestampNow() - traj_start_t_) / 1000000.0;
    } else {
        std::cerr << "WARNING: no TI trajectory with UNSET start t.  Good luck." << std::endl;
        t = 0;
    }

    double dist = current_traj_->ClosestObstacleInRemainderOfTrajectory(*octomap_, body_to_local, t);

    if (dist > safe_distance_ || dist < 0) {
        // we're still OK
        return false;
    }

    // we might hit something!
    // get a better trajectory if possible!

    double new_dist;
    const Trajectory *traj;

    std::tie(new_dist, traj) = trajlib_->FindFarthestTrajectory(*octomap_, body_to_local, safe_distance_);

    if (new_dist > dist) {
        // this trajectory is better than the old one!

        RequestTrajectory(*traj);
        return true;
    } else {
        return false;
    }
}


/**
 * Sends an LCM message requesting the controller to switch to a new
 * trajectory.  Also updates internal state about which trajectory we
 * are running.  Transitions the state machine to the ExecutingTrajectory
 * state
 */
void StateMachineControl::RequestTrajectory(const Trajectory &traj) {

    lcmt::tvlqr_controller_action msg;

    msg.timestamp = GetTimestampNow();
    msg.trajectory_number = traj.GetTrajectoryNumber();


    current_traj_ = &traj;
    traj_start_t_ = msg.timestamp;

    lcm_->publish(tvlqr_action_out_channel_, &msg);

}


int main(int argc,char** argv) {

    bool ttl_one = false;

    std::string trajectory_dir = "";
    std::string pose_channel = "STATE_ESTIMATOR_POSE";
    std::string stereo_channel = "stereo";

    std::string tvlqr_action_out_channel = "tvlqr-action-out";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(trajectory_dir, "d", "trajectory-dir", "Directory containing CSV files with trajectories.", true);
    parser.add(pose_channel, "p", "pose-channel", "LCM channel to listen for pose messages on.");
    parser.add(stereo_channel, "e", "stereo-channel", "LCM channel to listen to stereo messages on.");
    parser.add(tvlqr_action_out_channel, "o", "tvlqr-out-channel", "LCM channel to publish which TVLQR trajectory is running on.");

    parser.parse();


    std::string lcm_url;
    // create an lcm instance
    if (ttl_one) {
        lcm_url = "udpm://239.255.76.67:7667?ttl=1";
    } else {
        lcm_url = "udpm://239.255.76.67:7667?ttl=0";
    }
    lcm::LCM lcm(lcm_url);

    if (!lcm.good()) {
        std::cerr << "LCM creation failed." << std::endl;
        return 1;
    }

    // get parameter server
    BotParam *param = bot_param_new_from_server(lcm.getUnderlyingLCM(), 0);
    BotFrames *bot_frames = bot_frames_get_global(lcm.getUnderlyingLCM(), param);

    double dist_threshold = bot_param_get_double_or_fail(param, "obstacle_avoidance.safe_distance_threshold");
    int start_traj_num = bot_param_get_int_or_fail(param, "tvlqr_controller.stable_controller");

    StateMachineControl fsm_control(&lcm, trajectory_dir, bot_frames, dist_threshold, start_traj_num, tvlqr_action_out_channel);

    // subscribe to LCM channels
    lcm.subscribe(pose_channel, &StateMachineControl::ProcessImuMsg, &fsm_control);
    lcm.subscribe(stereo_channel, &StateMachineControl::ProcessStereoMsg, &fsm_control);


    while (0 == lcm.handle());

    return 0;
}
