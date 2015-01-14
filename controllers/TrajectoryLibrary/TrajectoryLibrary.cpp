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

    while ((dp = readdir(dirp)) != NULL) {
        string this_file = dp->d_name;

        if (this_file.length() > 4 && this_file.compare(this_file.length()-6, 6, "-x.csv") == 0)
        {
            // found a .csv file
            // load a trajectory

            Trajectory this_traj(dirname + this_file.substr(0, this_file.length()-6));

            // add it to the library
            traj_vector_.push_back(this_traj);

            count ++;
        }
    }

    //(void)closedir(dirp);

    cout << "Loaded " << count << " trajectories." << endl;

    if (count > 0) {
        return true;
    }
    return false;
}

void TrajectoryLibrary::Print() {
    for (auto traj : traj_vector_) {

        traj.Print();

    }
}

Trajectory* TrajectoryLibrary::GetTrajectoryByNumber(int number) {
    for (unsigned int i = 0; i < traj_vector_.size(); i++) {
        if (traj_vector_.at(i).GetTrajectoryNumber() == number) {
            return &traj_vector_.at(i);
        }
    }
    return NULL;
}

Trajectory* TrajectoryLibrary::FindFarthestTrajectory(OcTree *octree, BotTrans *bodyToLocal, bot_lcmgl_t *lcmgl) {

    /*
    double minProbability = -1;
    Trajectory *farthestTraj = NULL;

    if (lcmgl != NULL) {
        bot_lcmgl_push_matrix(lcmgl);

    }

    // for each point in each trajectory, find the point that is closest in the octree
    for (int i=0; i<int(traj_vector_.size()); i++) {


        double thisTrajProbability = 0;
        // for each trajectory, look at each point
        for (int j=0; j<int(traj_vector_[i].xpoints_.size()); j++) {
            // now we are looking at a single point in a trajectorybot_lcmgl_t *lcmgl

            double transformedPoint[3];

            traj_vector_[i].GetTransformedPoint(j, bodyToLocal, transformedPoint);

            point3d query (transformedPoint[0], transformedPoint[1], transformedPoint[2]);

            OcTreeNode* result = octree->search(query);


            if (result != NULL) {
                //cout << "occupancy probability at " << query << ":\t " << result->getOccupancy() << endl;
                thisTrajProbability += result->getOccupancy();
            } else {
                //cout << "occupancy probability at " << query << ":\t is unknown" << endl;
                //break;
            }


        }

        if (lcmgl != NULL) {
            //traj_vector_[i].PlotTransformedTrajectory(lcmgl, bodyToLocal);
        }

        if (minProbability == -1 || thisTrajProbability < minProbability) {
            minProbability = thisTrajProbability;
            farthestTraj = &traj_vector_[i];
        }
    }

    if (lcmgl != NULL) {
        // plot the best trajectory
        if (farthestTraj != NULL) {
            bot_lcmgl_color3f(lcmgl, 1, 0, 0);
            farthestTraj->PlotTransformedTrajectory(lcmgl, bodyToLocal);
        }

        bot_lcmgl_pop_matrix(lcmgl);
        bot_lcmgl_switch_buffer(lcmgl);
    }

    return farthestTraj;
    */
    return NULL;
}

