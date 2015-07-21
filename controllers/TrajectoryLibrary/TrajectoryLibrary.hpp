#ifndef TRAJECTORYLIB_TRAJLIB_HPP
#define TRAJECTORYLIB_TRAJLIB_HPP

/*
 * Trajectory library class
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <sstream>

#include <dirent.h>
#include <tuple>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>


#include "Trajectory.hpp"
#include "../../estimators/StereoOctomap/StereoOctomap.hpp"


class TrajectoryLibrary
{

    public:
        TrajectoryLibrary();

        Trajectory* GetTrajectoryByNumber(int number);

        bool LoadLibrary(std::string dirname);  // loads a trajectory from a directory of .csv files

        std::tuple<double, Trajectory*> FindFarthestTrajectory(const StereoOctomap *octomap, const BotTrans *bodyToLocal, double threshold, bot_lcmgl_t *lcmgl = nullptr);

        void Print();



    private:
        vector<Trajectory> traj_vector_;

};

#endif
