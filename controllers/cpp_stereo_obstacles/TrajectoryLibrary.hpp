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
#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>


#include "Trajectory.hpp"

using namespace std;
using namespace octomap;


class TrajectoryLibrary
{

    public:
        TrajectoryLibrary();
        
        bool LoadLibrary(string dirname);  // loads a trajectory from a directory of .csv files
        
        Trajectory* FindFarthestTrajectory(OcTree *octree, BotTrans *bodyToLocal, bot_lcmgl_t *lcmgl = NULL);
        
        
        
    private:
        vector<Trajectory> trajVector;

};

#endif
