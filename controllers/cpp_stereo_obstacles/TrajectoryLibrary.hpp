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

#include "Trajectory.hpp"

using namespace std;


class TrajectoryLibrary
{

    public:
        TrajectoryLibrary();
        
        bool LoadLibrary(string dirname);  // loads a trajectory from a .csv file
        
        
        
        
    private:
        vector<Trajectory*> trajVector;


};

#endif
