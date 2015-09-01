#include "RcSwitchDispatch.hpp"
#include "../../externals/ConciseArgs.hpp"

RcSwitchDispatch::RcSwitchDispatch(lcm::LCM *lcm, std::string rc_trajectory_commands_channel, std::string stereo_channel, std::string state_machine_go_autonomous_channel, std::string stereo_control_channel, std::string arm_for_takeoff_channel) {
    lcm_ = lcm;
    rc_trajectory_commands_channel_ = rc_trajectory_commands_channel;
    stereo_channel_ = stereo_channel;
    state_machine_go_autonomous_channel_ = state_machine_go_autonomous_channel;
    stereo_control_channel_ = stereo_control_channel;
    arm_for_takeoff_channel_ = arm_for_takeoff_channel;

    param_ = bot_param_new_from_server(lcm_->getUnderlyingLCM(), 0);
    bot_frames_ = bot_frames_new(lcm_->getUnderlyingLCM(), param_);

    if (param_ == nullptr) {
        std::cerr << "ERROR: failed to get a parameter server." << std::endl;
        exit(1);
    }

    stable_traj_num_ = bot_param_get_int_or_fail(param_, "tvlqr_controller.stable_controller");

    num_switch_positions_ = bot_param_get_int_or_fail(param_, "rc_switch_action.number_of_switch_positions");
    num_trajs_ = bot_param_get_int_or_fail(param_, "rc_switch_action.number_of_trajectories");
    num_stereo_actions_ = bot_param_get_int_or_fail(param_, "rc_switch_action.number_of_stereo_actions");
    int num_unused = bot_param_get_int_or_fail(param_, "rc_switch_action.number_of_unused_slots");

    if (num_trajs_ + num_stereo_actions_ + num_unused  + 2 != num_switch_positions_) { // + 1 for stop stereo
        std::cerr << "ERROR: num_trajs (" << num_trajs_ << ") + num_stereo (" << num_stereo_actions_ << ") + other actions (2) + num_unused (" << num_unused << ") != num_switch_positions (" << num_switch_positions_ << ")" << std::endl;
        exit(1);
    }

    bot_param_get_int_array_or_fail(param_, "rc_switch_action.switch_rc_us", switch_rc_us_, num_switch_positions_);
    bot_param_get_int_array_or_fail(param_, "rc_switch_action.trajectories", trajectory_mapping_, num_trajs_);
    bot_param_get_int_array_or_fail(param_, "rc_switch_action.stereo_actions", stereo_mapping_, num_stereo_actions_);
    stop_stereo_pos_ = bot_param_get_int_or_fail(param_, "rc_switch_action.stop_stereo_recording_switch_position");
    arm_for_takeoff_pos_ = bot_param_get_int_or_fail(param_, "rc_switch_action.arm_for_takeoff_switch_position");

    stabilization_mode_ = false;
}

void RcSwitchDispatch::ProcessRcMsg(const lcm::ReceiveBuffer *rbus, const std::string &chan, const lcmt::rc_switch_action *msg) {
    // new message has come in from the RC controller -- figure out what to do

    int switch_action = ServoMicroSecondsToSwitchPosition(msg->pulse_us);

    if (switch_action < 0) {
        // stabilization mode
        if (stabilization_mode_ == false) {
            SendTrajectoryRequest(stable_traj_num_);
            stabilization_mode_ = true;
        }
    } else {
        // not stabilization mode could be:
        //      - run a different trajectory
        //      - send stereo messages

        stabilization_mode_ = false;

        if (switch_action < num_trajs_) {
            SendTrajectoryRequest(trajectory_mapping_[switch_action]);
        } else if (switch_action < num_trajs_ + num_stereo_actions_) {
            SendGoAutonomousMsg();
            SendStereoMsg(stereo_mapping_[switch_action - num_trajs_]);
        } else if (switch_action == arm_for_takeoff_pos_) {
            SendArmForTakeoffMsg();
        } else if (switch_action == stop_stereo_pos_) {
            SendStereoWriteToDiskMsg();
        } else {
            std::cerr << "WARNING: unused action requested: " << switch_action << std::endl;
        }
    }
}

void RcSwitchDispatch::SendTrajectoryRequest(int traj_num) const {
    lcmt::tvlqr_controller_action trajectory_msg;
    trajectory_msg.timestamp = GetTimestampNow();

    trajectory_msg.trajectory_number = traj_num;

    lcm_->publish(rc_trajectory_commands_channel_, &trajectory_msg);
}

void RcSwitchDispatch::SendGoAutonomousMsg() const {
    lcmt::timestamp autonomous_msg;

    autonomous_msg.timestamp = GetTimestampNow();

    lcm_->publish(state_machine_go_autonomous_channel_, &autonomous_msg);
}

void RcSwitchDispatch::SendStereoMsg(int stereo_msg_num) const {
    if (stereo_msg_num < 0) {
        // do nothing
        return;
    }

    // send stereo messages

    std::vector<float> x, y, z;

    bool flag = true;

    if (stereo_msg_num == 0) {
        x = x_points0_;
        y = y_points0_;
        z = z_points0_;
    } else if (stereo_msg_num == 1) {
        x = x_points1_;
        y = y_points1_;
        z = z_points1_;
    } else if (stereo_msg_num == 2) {
        x = x_points2_;
        y = y_points2_;
        z = z_points2_;
    } else {
        flag = false;
        std::cerr << "WARNING: unknown stereo message request: " << stereo_msg_num << std::endl;
    }

    if (flag == true) {
        SendStereoManyPoints(x, y, z);
    }
}

void RcSwitchDispatch::SendStereoWriteToDiskMsg() const {
    lcmt::stereo_control msg;

    msg.timestamp = GetTimestampNow();
    msg.stereo_control = 0; // this is the stop, write to disk, then restart code

    lcm_->publish(stereo_control_channel_, &msg);
}

void RcSwitchDispatch::SendArmForTakeoffMsg() const {
    lcmt::timestamp arm_msg;
    arm_msg.timestamp = GetTimestampNow();
    lcm_->publish(state_machine_go_autonomous_channel_, &arm_msg);
}

void RcSwitchDispatch::DrakeToCameraFrame(double point_in[], double point_out[]) const {
    // figure out what this point (which is currently expressed in global coordinates
    // will be in local opencv coordinates

    BotTrans global_to_camera_trans;

    // we must update this every time because the transforms could change as the aircraft
    // moves
    bot_frames_get_trans(bot_frames_, "body", "opencvFrame", &global_to_camera_trans);

    bot_trans_apply_vec(&global_to_camera_trans, point_in, point_out);
}

void RcSwitchDispatch::SendStereoManyPoints(std::vector<float> x_in, std::vector<float> y_in, std::vector<float> z_in) const {
    lcmt::stereo msg;

    msg.timestamp = GetTimestampNow();

    std::vector<float> x, y, z;
    std::vector<unsigned char> grey;

    for (int i = 0; i < (int)x_in.size(); i++) {

        double this_point[3];

        this_point[0] = x_in[i];
        this_point[1] = y_in[i];
        this_point[2] = z_in[i];

        double point_transformed[3];
        DrakeToCameraFrame(this_point, point_transformed);

        ////std::cout << "Point: (" << point_transformed[0] << ", " << point_transformed[1] << ", " << point_transformed[2] << ")" << std::endl;

        x.push_back(point_transformed[0]);
        y.push_back(point_transformed[1]);
        z.push_back(point_transformed[2]);

        grey.push_back(0);
    }

    msg.x = x;
    msg.y = y;
    msg.z = z;

    msg.grey = grey;

    msg.number_of_points = x.size();
    msg.video_number = 0;
    msg.frame_number = 0;

    lcm_->publish("stereo", &msg);

}

int RcSwitchDispatch::ServoMicroSecondsToSwitchPosition(int servo_value) const {

    if (servo_value < 0) {
        // this is the stabilization mode
        return servo_value;
    }

    int min_delta = -1;
    int min_index = -1;

    for (int i = 0; i < num_switch_positions_; i++)
    {
        int delta = abs(servo_value - switch_rc_us_[i]);

        if (min_index < 0 || delta < min_delta) {
            min_delta = delta;
            min_index = i;
        }
    }

    return min_index;
}


int main(int argc,char** argv) {

    bool ttl_one = false;

    std::string rc_action_channel = "rc-switch-action";

    std::string rc_trajectory_commands_channel = "rc-trajectory-commands";
    std::string stereo_channel = "stereo";
    std::string state_machine_go_autonomous_channel = "state-machine-go-autonomous";
    std::string stereo_control_channel = "stereo-control";
    std::string arm_for_takeoff_channel = "arm-for-takeoff";


    ConciseArgs parser(argc, argv);
    parser.add(ttl_one, "t", "ttl-one", "Pass to set LCM TTL=1");
    parser.add(stereo_channel, "s", "stereo-channel", "LCM channel to listen to stereo messages on.");
    parser.add(rc_trajectory_commands_channel, "j", "rc-trajectory-commands-channel", "LCM channel to sned RC trajectory commands on.");
    parser.add(rc_action_channel, "r", "rc-action-channel", "LCM channel to listen for RC switch actions on.");
    parser.add(state_machine_go_autonomous_channel, "a", "state-machine-go-autonomous-channel", "LCM channel to send go-autonmous messages on.");
    parser.add(stereo_control_channel, "c", "stereo-control-channel", "LCM channel to send stereo control messages on.");
    parser.add(arm_for_takeoff_channel, "A", "arm-for-takeoff-channel", "LCM channel to send arm for takeoff messages on.");
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

    RcSwitchDispatch rc_dispatch(&lcm, rc_trajectory_commands_channel, stereo_channel, state_machine_go_autonomous_channel, stereo_control_channel, arm_for_takeoff_channel);

    // subscribe to LCM channels
    lcm.subscribe(rc_action_channel, &RcSwitchDispatch::ProcessRcMsg, &rc_dispatch);

    printf("Recieving LCM:\n\tRC Switch Actions: %s\nSending LCM:\n\tRC Trajectory Requests: %s\n\tStereo Messages: %s\n\tState Machine Go Autonomous: %s\n\tStereo Control: %s\n\tArm for Takeoff: %s\n", rc_action_channel.c_str(), rc_trajectory_commands_channel.c_str(), stereo_channel.c_str(), state_machine_go_autonomous_channel.c_str(), stereo_control_channel.c_str(), arm_for_takeoff_channel.c_str());

    while (0 == lcm.handle());

    return 0;
}
