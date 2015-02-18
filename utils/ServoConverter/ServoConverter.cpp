#include "ServoConverter.hpp"

ServoConverter::ServoConverter(BotParam *param) {

    rad_to_servo_.elevL_slope = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevL_slope");
    rad_to_servo_.elevL_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevL_y_intercept");

    rad_to_servo_.elevR_slope = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevR_slope");
    rad_to_servo_.elevR_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevR_y_intercept");

    rad_to_servo_.throttle_slope = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.throttle_slope");
    rad_to_servo_.throttle_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.throttle_y_intercept");

    servo_to_rad_.elevL_slope = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevL_slope");
    servo_to_rad_.elevL_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevL_y_intercept");

    servo_to_rad_.elevR_slope = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevR_slope");
    servo_to_rad_.elevR_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevR_y_intercept");

    servo_to_rad_.throttle_slope = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.throttle_slope");
    servo_to_rad_.throttle_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.throttle_y_intercept");


    elevL_min_ = bot_param_get_int_or_fail(param, "servo_commands.elevL_min");
    elevL_max_ = bot_param_get_int_or_fail(param, "servo_commands.elevL_max");

    elevR_min_ = bot_param_get_int_or_fail(param, "servo_commands.elevR_min");
    elevR_max_ = bot_param_get_int_or_fail(param, "servo_commands.elevR_max");

    throttle_min_ = bot_param_get_int_or_fail(param, "servo_commands.throttle_min");
    throttle_max_ = bot_param_get_int_or_fail(param, "servo_commands.throttle_max");

    elevL_trim_ = bot_param_get_int_or_fail(param, "servo_commands.elevL_trim");
    elevR_trim_ = bot_param_get_int_or_fail(param, "servo_commands.elevR_trim");
    throttle_trim_ = bot_param_get_int_or_fail(param, "servo_commands.throttle_trim");

}

Eigen::Vector3i ServoConverter::RadiansToServoCommands(Eigen::Vector3d commands) const {

    Eigen::Vector3i output;

    output(0) = round(commands(0) * rad_to_servo_.elevL_slope + rad_to_servo_.elevL_y_intercept);
    output(1) = round(commands(1) * rad_to_servo_.elevR_slope + rad_to_servo_.elevR_y_intercept);
    output(2) = round(commands(2) * rad_to_servo_.throttle_slope + rad_to_servo_.throttle_y_intercept);

    return MinMaxCommands(output);

}

Eigen::Vector3d ServoConverter::ServoCommandsToRadians(Eigen::Vector3i commands) const {

    Eigen::Vector3d output;

    commands = MinMaxCommands(commands);

    output(0) = commands(0) * servo_to_rad_.elevL_slope + servo_to_rad_.elevL_y_intercept;
    output(1) = commands(1) * servo_to_rad_.elevR_slope + servo_to_rad_.elevR_y_intercept;
    output(2) = commands(2) * servo_to_rad_.throttle_slope + servo_to_rad_.throttle_y_intercept;

    return output;

}

Eigen::Vector3i ServoConverter::MinMaxCommands(Eigen::Vector3i commands) const {

    Eigen::Vector3i output;

    output(0) = max(commands(0), elevL_min_);
    output(0) = min(output(0), elevL_max_);


    output(1) = max(commands(1), elevR_min_);
    output(1) = min(output(1), elevR_max_);

    output(2) = max(commands(2), throttle_min_);
    output(2) = min(output(2), throttle_max_);

    return output;

}

Eigen::Vector3i ServoConverter::GetTrimCommands() const {
    Eigen::Vector3i output;

    output(0) = elevL_trim_;
    output(1) = elevR_trim_;
    output(2) = throttle_trim_;

    return output;
}

