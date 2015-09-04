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
        TrajectoryLibrary(double ground_safety_distance);

        const Trajectory* GetTrajectoryByNumber(int number) const;

        int GetNumberTrajectories() const { return int(traj_vec_.size()); }

        bool LoadLibrary(std::string dirname, bool quiet = false);  // loads a trajectory from a directory of .csv files

        std::tuple<double, const Trajectory*> FindFarthestTrajectory(const StereoOctomap &octomap, const BotTrans &bodyToLocal, double threshold, bot_lcmgl_t* lcmgl = nullptr) const;

        void Print() const;
        void Draw(lcm_t *lcm, const BotTrans *transform = nullptr) const;



    private:
        std::vector<Trajectory> traj_vec_;
        double ground_safety_distance_;

};

#endif
