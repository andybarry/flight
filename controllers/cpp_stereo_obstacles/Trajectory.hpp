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

using namespace std;

class Trajectory
{

    public:
        Trajectory(string filename); // loads a trajectory from a .csv file
        
        vector<double> GetPoint(double t);
        
        int GetDimension() { return dimension; }
        int GetUDimension() { return udimension; }
        double GetTLength() { return tlength; }
        
        void print();
        
        
    private:
        int dimension; // state space dimension
        int udimension; // control input dimension
        double tlength; // length in time
        vector<vector<double>> xpoints;
        vector<vector<double>> upoints;

        void LoadXFromCSV( const std::string& filename);
        void LoadUFromCSV( const std::string& filename);

};

#endif
