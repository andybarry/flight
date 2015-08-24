#ifndef TRAJECTORY_TRAJLIB_HPP
#define TRAJECTORY_TRAJLIB_HPP

/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013-2015
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_vis/gl_util.h>
#include "gtest/gtest.h"
#include "../../utils/utils/RealtimeUtils.hpp"

#include "../../externals/csvparser/csvparser.h"
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"

#include <Eigen/Core>

class Trajectory
{

    public:
        Trajectory();
        Trajectory(std::string filename_prefix, bool quiet = false); // loads a trajectory from a .csv file

        void LoadTrajectory(std::string filename_prefix, bool quiet = false);

        int GetDimension() const { return dimension_; }
        int GetUDimension() const { return udimension_; }
        int GetTrajectoryNumber() const { return trajectory_number_; }
        double GetDT() const { return dt_; }

        void GetXyzYawTransformedPoint(double t, const BotTrans &transform, double *xyz) const;
        void Draw(bot_lcmgl_t *lcmgl, const BotTrans *transform = nullptr) const;

        int GetIndexAtTime(double t) const;
        double GetTimeAtIndex(int index) const { return xpoints_(index, 0); }

        double GetMaxTime() const { return xpoints_(xpoints_.rows() - 1, 0); }

        bool IsTimeInvariant() const { return upoints_.rows() == 1; }

        int GetNumberOfPoints() const { return int(xpoints_.rows()); }

        Eigen::VectorXd GetState(double t) const;
        Eigen::VectorXd GetUCommand(double t) const;
        Eigen::MatrixXd GetGainMatrix(double t) const;

        Eigen::MatrixXd GetXpoints() const { return xpoints_; }

        double ClosestObstacleInRemainderOfTrajectory(const StereoOctomap &octomap, const BotTrans &body_to_local, double current_t) const;

        void Print() const;

        double GetMinimumAltitude() { return min_altitude_; }



    private:

        Eigen::MatrixXd xpoints_;
        Eigen::MatrixXd upoints_;

        Eigen::MatrixXd kpoints_;
        Eigen::MatrixXd affine_points_;

        double dt_;
        double min_altitude_;


        int dimension_; // state space dimension
        int udimension_; // control input dimension

        int trajectory_number_;
        std::string filename_prefix_;

        void LoadMatrixFromCSV(const std::string& filename, Eigen::MatrixXd &matrix, bool quiet = false);

        int GetNumberOfLines(std::string filename) const;

};

#endif
