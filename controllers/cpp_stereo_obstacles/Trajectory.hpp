#ifndef TRAJECTORY_TRAJLIB_HPP
#define TRAJECTORY_TRAJLIB_HPP

/*
 * Trajectory class representing a trajectory in state space
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
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

using namespace std;

class Trajectory
{

    public:
        Trajectory();
        Trajectory(string filename, bool quiet = false); // loads a trajectory from a .csv file

        void LoadTrajectory(string filename, bool quiet = false);

        vector<double> GetPoint(double t);

        int GetDimension() { return dimension_; }
        int GetUDimension() { return udimension_; }
        int GetTrajectoryNumber() { return trajectory_number_; }

        void Print();

        void GetTransformedPoint(int index, BotTrans *transform, double *xyz);
        void PlotTransformedTrajectory(bot_lcmgl_t *lcmgl, BotTrans *transform);


        // returns the distance to the closest point on the trajectory
        // could optimize this with cover trees?
        //double DistanceToPoint(double x, double y, double z);

        vector<vector<double>> xpoints_;
        vector<vector<double>> upoints_;


    private:


        int dimension_; // state space dimension
        int udimension_; // control input dimension
        int trajectory_number_;
        string filename_;

        void LoadXFromCSV( const std::string& filename);
        void LoadUFromCSV( const std::string& filename);

};

#endif
