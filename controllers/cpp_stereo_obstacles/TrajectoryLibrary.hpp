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


#include "Trajectory.hpp"

using namespace std;
using namespace octomap;


class TrajectoryLibrary
{

    public:
        TrajectoryLibrary();
        
        bool LoadLibrary(string dirname);  // loads a trajectory from a directory of .csv files
        
        void FindFarthestTrajectory(OcTree octree, Trajectory *fathestTraj);
        
        
    private:
        vector<Trajectory> trajVector;


};

#endif
