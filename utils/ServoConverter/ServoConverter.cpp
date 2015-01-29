#include "ServoConverter.hpp"

ServoConverter::ServoConverter(BotParam *param) {
    rad_to_servo.elevL_slope = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevL_slope");
    rad_to_servo.elevL_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevL_y_intercept");

    rad_to_servo.elevR_slope = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevR_slope");
    rad_to_servo.elevR_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.elevR_y_intercept");

    rad_to_servo.throttle_slope = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.throttle_slope");
    rad_to_servo.throttle_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.radians_to_servo.throttle_y_intercept");

    servo_to_rad.elevL_slope = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevL_slope");
    servo_to_rad.elevL_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevL_y_intercept");

    servo_to_rad.elevR_slope = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevR_slope");
    servo_to_rad.elevR_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.elevR_y_intercept");

    servo_to_rad.throttle_slope = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.throttle_slope");
    servo_to_rad.throttle_y_intercept = bot_param_get_double_or_fail(param, "servo_commands.servo_to_radians.throttle_y_intercept");

}

Eigen::Vector3i ServoConverter::RadiansToServoCommands(Eigen::Vector3d commands) {

    Eigen::Vector3i output;

    output(0) = round(commands(0) * rad_to_servo.elevL_slope + rad_to_servo.elevL_y_intercept);
    output(1) = round(commands(1) * rad_to_servo.elevR_slope + rad_to_servo.elevR_y_intercept);
    output(2) = round(commands(2) * rad_to_servo.throttle_slope + rad_to_servo.throttle_y_intercept);

    return output;

}

Eigen::Vector3d ServoConverter::ServoCommandsToRadians(Eigen::Vector3i commands) {

    Eigen::Vector3d output;

    output(0) = commands(0) * servo_to_rad.elevL_slope + servo_to_rad.elevL_y_intercept;
    output(1) = commands(1) * servo_to_rad.elevR_slope + servo_to_rad.elevR_y_intercept;
    output(2) = commands(2) * servo_to_rad.throttle_slope + servo_to_rad.throttle_y_intercept;

    return output;

}

