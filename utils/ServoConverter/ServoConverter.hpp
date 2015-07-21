#ifndef SERVO_CONVERTER_HPP
#define SERVO_CONVERTER_HPP

/*
 * Servo to radian converter
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2015
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>


#include "../../LCM/mav_pose_t.h"

#include <Eigen/Core>

#include "gtest/gtest.h"

#include <bot_param/param_client.h>

struct ServoConfig {
    double elevL_slope;
    double elevL_y_intercept;

    double elevR_slope;
    double elevR_y_intercept;

    double throttle_slope;
    double throttle_y_intercept;
};


class ServoConverter {

    public:
        ServoConverter(BotParam *param);

        Eigen::Vector3i RadiansToServoCommands(Eigen::Vector3d commands) const;

        Eigen::Vector3d ServoCommandsToRadians(Eigen::Vector3i commands) const;

        Eigen::Vector3i MinMaxCommands(Eigen::Vector3i commands) const;

        Eigen::Vector3i GetTrimCommands() const;


    private:
        ServoConfig rad_to_servo_;
        ServoConfig servo_to_rad_;

        int elevL_min_;
        int elevL_max_;

        int elevR_min_;
        int elevR_max_;

        int throttle_min_;
        int throttle_max_;

        int elevL_trim_;
        int elevR_trim_;
        int throttle_trim_;




};


#endif
