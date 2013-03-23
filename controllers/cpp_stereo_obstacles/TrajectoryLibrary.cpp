/*
 * Trajectory library
 *
 * Author: Andrew Barry, <abarry@csail.mit.edu> 2013
 *
 */
 
#include "TrajectoryLibrary.hpp"

// Constructor that loads a trajectorys from a directory
TrajectoryLibrary::TrajectoryLibrary()
{
}

bool TrajectoryLibrary::LoadLibrary(string dirname)
{
    // if dirname does not end in "/", add a "/"
    if (dirname.back() != '/')
    {
        dirname.append("/");
    }
    
    
    // open the directory and find all the files that end in .csv
    DIR *dirp = opendir(dirname.c_str());
    struct dirent *dp;
    
    int count = 0;
    
    while ((dp = readdir(dirp)) != NULL)
    {
        string thisFile = dp->d_name;
        
        if (thisFile.length() > 4 && thisFile.compare(thisFile.length()-4, 4, ".csv") == 0 &&
        thisFile.compare(thisFile.length()-6, 6, "-u.csv") != 0)
        {
            // found a .csv file
            // load a trajectory
            
            Trajectory *thisTraj = new Trajectory(dirname + dp->d_name);
            
            // add it to the library
            trajVector.push_back(thisTraj);
            
            count ++;
        }
    }
    
    (void)closedir(dirp);

    cout << "Loaded " << count << " trajectories." << endl;

    if (count > 0)
    {
        return true;
    }
    return false;
}
