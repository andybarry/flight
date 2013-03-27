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
            
            Trajectory thisTraj(dirname + dp->d_name);
            
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

Trajectory* TrajectoryLibrary::FindFarthestTrajectory(OcTree *octree, BotTrans *bodyToLocal, bot_lcmgl_t *lcmgl)
{
    double minProbability = -1;
    Trajectory *farthestTraj = NULL;
    
    if (lcmgl != NULL)
    {
        bot_lcmgl_push_matrix(lcmgl);
        
    }
    
    // for each point in each trajectory, find the point that is closest in the octree
    for (int i=0; i<int(trajVector.size()); i++)
    {
        
    
        double thisTrajProbability = 0;
        // for each trajectory, look at each point
        for (int j=0; j<int(trajVector[i].xpoints.size()); j++)
        {
            // now we are looking at a single point in a trajectorybot_lcmgl_t *lcmgl
            
            double transformedPoint[3];
            
            trajVector[i].GetTransformedPoint(j, bodyToLocal, transformedPoint);
            
            point3d query (transformedPoint[0], transformedPoint[1], transformedPoint[2]);
            
            OcTreeNode* result = octree->search(query);
            
            
            if (result != NULL)
            {
                //cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << endl;
                thisTrajProbability += result->getOccupancy();
            } else {
                //cout << "occupancy probability at " << query << ":\t is unknown" << endl;
                //break;
            }
            
            
        }
        
        if (lcmgl != NULL)
        {
            //trajVector[i].PlotTransformedTrajectory(lcmgl, bodyToLocal);
        }
        if (minProbability == -1 || thisTrajProbability < minProbability)
        {
            minProbability = thisTrajProbability;
            farthestTraj = &trajVector[i];
        }
    }
    
    if (lcmgl != NULL)
    {
        // plot the best trajectory
        if (farthestTraj != NULL)
        {
            bot_lcmgl_color3f(lcmgl, 1, 0, 0);
            farthestTraj->PlotTransformedTrajectory(lcmgl, bodyToLocal);
        }
        
        bot_lcmgl_pop_matrix(lcmgl);
        bot_lcmgl_switch_buffer(lcmgl);
    }
    
    return farthestTraj;
}

