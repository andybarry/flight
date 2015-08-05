#include "StateMachineControl.hpp"

StateMachineControl::StateMachineControl(lcm::LCM *lcm, std::string traj_dir, BotFrames *bot_frames, double dist_threshold, int stable_traj_num, std::string tvlqr_action_out_channel) : fsm_(*this) {
    lcm_ = lcm;
    bot_frames_ = bot_frames;

    octomap_ = new StereoOctomap(bot_frames_);

    trajlib_ = new TrajectoryLibrary();

    if (trajlib_->LoadLibrary(traj_dir, true) == false) {
        std::cerr << "ERROR: Failed to load trajectory library." << std::endl;
        exit(1);
    }

    current_traj_ = trajlib_->GetTrajectoryByNumber(stable_traj_num);
    stable_traj_ = current_traj_;

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

        // transition the state machine into a new trajectory
        fsm_.NewTrajectory(*traj);

        return true;
    } else {
        return false;
    }
}

/**
 * Checks if the current trajectory has run it's full length
 * and we should switch to something else.
 */
bool StateMachineControl::CheckTrajectoryExpired() {

    if (current_traj_->IsTimeInvariant()) {
        return false; // TODO
    }

    if (GetTimestampNow() > traj_start_t_ + current_traj_->GetMaxTime() * 1000000.0) {
        return true;
    } else {
        return false;
    }
}


/**
 * Sends an LCM message requesting the controller to switch to a new
 * trajectory.  Also updates internal state about which trajectory we
 * are running.
 */
void StateMachineControl::RequestTrajectory(const Trajectory &traj) {

    lcmt::tvlqr_controller_action msg;

    msg.timestamp = GetTimestampNow();
    msg.trajectory_number = current_traj_->GetTrajectoryNumber();


    traj_start_t_ = msg.timestamp;

    lcm_->publish(tvlqr_action_out_channel_, &msg);
}

