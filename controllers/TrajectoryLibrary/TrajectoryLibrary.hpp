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

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>


#include "Trajectory.hpp"

using namespace std;


class TrajectoryLibrary
{

    public:
        TrajectoryLibrary();

        Trajectory* GetTrajectoryByNumber(int number);

        bool LoadLibrary(string dirname);  // loads a trajectory from a directory of .csv files

        //Trajectory* FindFarthestTrajectory(const OcTree *octree, const BotTrans *bodyToLocal, bot_lcmgl_t *lcmgl = NULL);

        void Print();



    private:
        vector<Trajectory> traj_vector_;

};

#endif
