#include "StateMachineControl.hpp"

StateMachineControl::StateMachineControl(lcm::LCM *lcm, std::string traj_dir, std::string tvlqr_action_out_channel, bool visualization) : fsm_(*this) {
    lcm_ = lcm;

    param_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    bot_frames_ = bot_frames_new(lcm_->getUnderlyingLCM(), param_);

    safe_distance_ = bot_param_get_double_or_fail(param_, "obstacle_avoidance.safe_distance_threshold");
    min_improvement_to_switch_trajs_ = bot_param_get_double_or_fail(param_, "obstacle_avoidance.min_improvement_to_switch_trajs");

    if (min_improvement_to_switch_trajs_ <= 0) {
        std::cerr << "ERROR: obstacle_avoidance.min_improvement_to_switch_trajs must be greater than 0." << std::endl;
        exit(1);
    }

    int stable_traj_num = bot_param_get_int_or_fail(param_, "tvlqr_controller.stable_controller");

    octomap_ = new StereoOctomap(bot_frames_);

    trajlib_ = new TrajectoryLibrary();

    if (trajlib_->LoadLibrary(traj_dir, true) == false) {
        std::cerr << "ERROR: Failed to load trajectory library." << std::endl;
        exit(1);
    }

    current_traj_ = trajlib_->GetTrajectoryByNumber(stable_traj_num);

    if (current_traj_ == nullptr) {
        std::cerr << "ERROR: Stable trajectory (# " << stable_traj_num << ") does not exist." << std::endl;
        exit(1);
    }

    stable_traj_ = current_traj_;
    next_traj_ = current_traj_;

    if (stable_traj_->IsTimeInvariant() == false) {
        std::cerr << "ERROR: stable trajectory is not time invariant." << std::endl;
        exit(1);
    }

    need_imu_update_ = false;
    visualization_ = visualization;

    tvlqr_action_out_channel_ = tvlqr_action_out_channel;

}

StateMachineControl::~StateMachineControl() {
    delete octomap_;
    delete trajlib_;
}

void StateMachineControl::SetNextTrajectoryByNumber(int traj_num) {
    const Trajectory *traj = trajlib_->GetTrajectoryByNumber(traj_num);

    if (traj != nullptr) {
        next_traj_ = traj;
    } else {
        std::cerr << "WARNING: trajectory # " << traj_num << " is NULL!" << std::endl;
    }
}

void StateMachineControl::ProcessImuMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const mav::pose_t *msg) {
    need_imu_update_ = true;
    last_imu_msg_ = *msg;
}

void StateMachineControl::DoDelayedImuUpdate() {
    if (need_imu_update_) {
        fsm_.ImuUpdate(last_imu_msg_);
        need_imu_update_ = false;

        if (visualization_) {
            octomap_->Draw(lcm_->getUnderlyingLCM());
            PublishDebugMsg("StateMachineControl: visualization");
        }
    }
}

void StateMachineControl::ProcessStereoMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::stereo *msg) {
    octomap_->ProcessStereoMessage(msg);
}

void StateMachineControl::ProcessRcTrajectoryMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::tvlqr_controller_action *msg) {
    std::cout << "got trajectory request message" << std::endl;
    fsm_.SingleTrajectoryRequest(msg->trajectory_number);
}

void StateMachineControl::ProcessGoAutonomousMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::timestamp *msg) {
    fsm_.AutonomousMode();
}

void StateMachineControl::PublishDebugMsg(std::string debug_str) const {
    lcmt::debug debug_msg;
    debug_msg.utime = GetTimestampNow();
    debug_msg.debug = debug_str;

    lcm_->publish("debug", &debug_msg);
}

void StateMachineControl::SetBestTrajectory() {
    std::cout << "set BEST traj" << std::endl;
    BotTrans body_to_local;
    bot_frames_get_trans(bot_frames_, "body", "local", &body_to_local);

    double dist;
    const Trajectory *traj;

    std::tie(dist, traj) = trajlib_->FindFarthestTrajectory(*octomap_, body_to_local, safe_distance_);

    SetNextTrajectory(*traj);
}

bool StateMachineControl::BetterTrajectoryAvailable() {
    // search for an obstacle in the path
    BotTrans body_to_local;
    bot_frames_get_trans(bot_frames_, "body", "local", &body_to_local);

    double t;

    if (current_traj_->IsTimeInvariant()) {
        t = 0;
    } else if (traj_start_t_ > 0) {
        t = (GetTimestampNow() - traj_start_t_) / 1000000.0;
    } else {
        std::cerr << "WARNING: TV trajectory with UNSET start t.  Good luck." << std::endl;
        t = 0;
    }

    double dist = current_traj_->ClosestObstacleInRemainderOfTrajectory(*octomap_, body_to_local, t);

    if (dist > safe_distance_ || dist < 0) {
        // we're still OK
        return false;
    }

    double new_dist;
    const Trajectory *traj;
    std::tie(new_dist, traj) = trajlib_->FindFarthestTrajectory(*octomap_, body_to_local, safe_distance_);

    double dist_diff = new_dist - dist;

    if (dist_diff > min_improvement_to_switch_trajs_) {
        // this trajectory is better than the old one!
        std::cout << "INTERRUPT: " << current_traj_->GetTrajectoryNumber() << " -> " << traj->GetTrajectoryNumber() << " dist = " << new_dist << std::endl;
        SetNextTrajectory(*traj);
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
void StateMachineControl::RequestNewTrajectory() {
    lcmt::tvlqr_controller_action msg;

    msg.timestamp = GetTimestampNow();
    msg.trajectory_number = next_traj_->GetTrajectoryNumber();

    current_traj_ = next_traj_;
    traj_start_t_ = msg.timestamp;

    std::cout << "Requesting trajectory: " << msg.trajectory_number << std::endl;

    lcm_->publish(tvlqr_action_out_channel_, &msg);

    if (visualization_) {
        // draw the trajectory via lcmgl
        BotTrans body_to_local;
        bot_frames_get_trans(bot_frames_, "body", "local", &body_to_local);

        bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm_->getUnderlyingLCM(), "StateMachineControl Trajectory");
        bot_lcmgl_color3f(lcmgl, 0, 0, 1);

        current_traj_->Draw(lcmgl, &body_to_local);
        bot_lcmgl_switch_buffer(lcmgl);
        bot_lcmgl_destroy(lcmgl);

        PublishDebugMsg("StateMachineControl: visualization");
    }
}

/**
 * Checks if the current trajectory has run it's full length
 * and we should switch to something else.
 */
bool StateMachineControl::CheckTrajectoryExpired() {

    if (current_traj_->GetTrajectoryNumber() == stable_traj_->GetTrajectoryNumber()) {
        // stable trajectory never times out
        return false;
    }

    if (GetTimestampNow() > traj_start_t_ + current_traj_->GetMaxTime() * 1000000.0) {
        std::cout << "Trajectory # " << current_traj_->GetTrajectoryNumber() << " completed." << std::endl;
        return true;
    } else {
        return false;
    }
}




