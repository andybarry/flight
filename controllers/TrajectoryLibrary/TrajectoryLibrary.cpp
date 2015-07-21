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

bool TrajectoryLibrary::LoadLibrary(std::string dirname)
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
        std::string this_file = dp->d_name;

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

    std::cout << "Loaded " << count << " trajectorie(s)." << std::endl;

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

/**
 * Finds the first Trajectory that is at least "threshold" distance away from any obstacle.
 * In the case  that there is no such trajectory, returns the trajectory that is furthest from obstacles.
 *
 * @param octomap obstacle map
 * @param bodyToLocal tranform for the aircraft that describes where we are in the map
 * @param threshold minimum safe distance for the aircraft
 * @param trajectory_out pointer that will be set to the best trajectory
 * @param (optional) lcmgl if not NULL, will draw debug data
 *
 * @retval the distance to the closest obstacle or -1 if there are no obstacles
 */
std::tuple<double, Trajectory*> TrajectoryLibrary::FindFarthestTrajectory(const StereoOctomap *octomap, const BotTrans *bodyToLocal, double threshold, bot_lcmgl_t *lcmgl) {

    Trajectory *farthest_traj = NULL;

    double traj_closest_dist = -1;

    if (lcmgl != NULL) {
        bot_lcmgl_push_matrix(lcmgl);
    }

    // for each point in each trajectory, find the point that is closest in the octree
    for (int i=0; i<int(traj_vector_.size()); i++) {


        double closest_obstacle_distance = -1;

        // for each trajectory, look at each point
        for (int j=0; j<int(traj_vector_[i].GetXpoints().size()); j++) {
            // now we are looking at a single point in a trajectorybot_lcmgl_t *lcmgl

            double transformedPoint[3];

            traj_vector_[i].GetTransformedPoint(j, bodyToLocal, transformedPoint);

            double distance_to_point = octomap->NearestNeighbor(transformedPoint);

            if (distance_to_point > 0) {
                if (distance_to_point < closest_obstacle_distance || closest_obstacle_distance < 0) {
                    closest_obstacle_distance = distance_to_point;
                }
            }

        }

        if (lcmgl != NULL) {
            //traj_vector_[i].PlotTransformedTrajectory(lcmgl, bodyToLocal);
        }

        if (traj_closest_dist == -1 || closest_obstacle_distance > traj_closest_dist) {
            traj_closest_dist = closest_obstacle_distance;
            farthest_traj = &traj_vector_[i];

            if (traj_closest_dist > threshold) {
                // we are satisfied with this one, run it!
                if (lcmgl != NULL) {
                    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
                    farthest_traj->PlotTransformedTrajectory(lcmgl, bodyToLocal);
                    bot_lcmgl_pop_matrix(lcmgl);
                    bot_lcmgl_switch_buffer(lcmgl);
                }

                return tuple<double, Trajectory*>(traj_closest_dist, farthest_traj);
            }
        }
    }

    if (lcmgl != NULL) {
        // plot the best trajectory
        if (farthest_traj != NULL) {
            bot_lcmgl_color3f(lcmgl, 1, 0, 0);
            farthest_traj->PlotTransformedTrajectory(lcmgl, bodyToLocal);
        }

        bot_lcmgl_pop_matrix(lcmgl);
        bot_lcmgl_switch_buffer(lcmgl);
    }

    return tuple<double, Trajectory*>(traj_closest_dist, farthest_traj);
}

