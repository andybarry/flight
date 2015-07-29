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

#include "../../externals/csvparser/csvparser.h"

#include <Eigen/Core>

class Trajectory
{

    public:
        Trajectory();
        Trajectory(std::string filename_prefix, bool quiet = false); // loads a trajectory from a .csv file

        void LoadTrajectory(std::string filename_prefix, bool quiet = false);

        Eigen::MatrixXd GetGainMatrix(double t) const;

        int GetDimension() const { return dimension_; }
        int GetUDimension() const { return udimension_; }
        int GetTrajectoryNumber() const { return trajectory_number_; }

        void Print() const;

        void GetTransformedPoint(double t, const BotTrans *transform, double *xyz) const;
        void PlotTransformedTrajectory(bot_lcmgl_t *lcmgl, const BotTrans *transform) const;

        int GetIndexAtTime(double t, bool use_rollout = false) const;
        double GetTimeAtIndex(int index) const { return xpoints_(index, 0); }

        double GetMaxTime() const;

        bool IsTimeInvariant() const { return GetMaxTime() == 0; }

        int GetNumberOfPoints() const { return int(xpoints_.rows()); }

        Eigen::VectorXd GetState(double t) const;
        Eigen::VectorXd GetUCommand(double t) const;
        Eigen::VectorXd GetRolloutState(double t) const;

        Eigen::MatrixXd GetXpoints() const { return xpoints_; }

        // returns the distance to the closest point on the trajectory
        // could optimize this with cover trees?
        //double DistanceToPoint(double x, double y, double z);




    private:

        Eigen::MatrixXd xpoints_;
        Eigen::MatrixXd upoints_;

        // in the case of a TI trajectory, we might want to
        // cache a simulation of the trajectory for some time
        // (loaded from CSV)
        Eigen::MatrixXd xpoints_rollout_;

        Eigen::MatrixXd kpoints_;
        Eigen::MatrixXd affine_points_;

        double dt_;


        int dimension_; // state space dimension
        int udimension_; // control input dimension

        int trajectory_number_;
        std::string filename_prefix_;

        void LoadMatrixFromCSV(const std::string& filename, Eigen::MatrixXd &matrix, bool quiet = false);

        int GetNumberOfLines(std::string filename) const;

};

#endif
