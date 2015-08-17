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

bool TrajectoryLibrary::LoadLibrary(std::string dirname, bool quiet) {
    // if dirname does not end in "/", add a "/"
    if (dirname.back() != '/')
    {
        dirname.append("/");
    }


    // open the directory and find all the files that end in .csv
    DIR *dirp = opendir(dirname.c_str());
    struct dirent *dp;

    if (dirp == nullptr) {
        std::cerr << "ERROR: no such directory: " << dirname << std::endl;
        return false;
    }

    vector<Trajectory> temp_traj;

    int count = 0;

    while ((dp = readdir(dirp)) != NULL) {
        std::string this_file = dp->d_name;

        if (this_file.length() > 4 && this_file.compare(this_file.length()-6, 6, "-x.csv") == 0) {
            // found a .csv file
            // load a trajectory

            Trajectory this_traj(dirname + this_file.substr(0, this_file.length()-6), quiet);

            temp_traj.push_back(this_traj);

            count ++;
        }
    }

    // now we have loaded everything into memory, so sort
    for (int i = 0; i < (int)temp_traj.size(); i++) {

        bool flag = false;
        for (auto traj : temp_traj) {
            if (traj.GetTrajectoryNumber() == i) {
                traj_vec_.push_back(traj);
                flag = true;
                break;
            }
        }
        if (flag == false) {
            std::cerr << "ERROR: missing trajectory #" << i << std::endl;

            return false;
        }
    }

    //(void)closedir(dirp);

    if (!quiet) {
        std::cout << "Loaded " << traj_vec_.size() << " trajectorie(s)" << std::endl;
    }

    if (traj_vec_.size() > 0) {
        return true;
    }
    return false;
}

void TrajectoryLibrary::Print() const {

    std::cout << "Time-varying trajectories" << std::endl << "------------------------" << std::endl;

    for (int i = 0; i < GetNumberTrajectories(); i++) {
        traj_vec_.at(i).Print();

    }
}

const Trajectory* TrajectoryLibrary::GetTrajectoryByNumber(int number) const {

    if (number >= GetNumberTrajectories()) {
        std::cerr << "WARNING: Requested trajectory #" << number << " which is not in library (library size = " << GetNumberTrajectories() << ")." << std::endl;
        return nullptr;
    } else {
        return &traj_vec_.at(number);
    }

}

/**
 * Finds the first Trajectory that is at least "threshold" distance away from any obstacle.
 * In the case  that there is no such trajectory, returns the trajectory that is furthest from obstacles.
 *
 * @param octomap obstacle map
 * @param body_to_local tranform for the aircraft that describes where we are in the map
 * @param threshold minimum safe distance for the aircraft
 * @param trajectory_out pointer that will be set to the best trajectory
 * @param (optional) lcmgl if not NULL, will draw debug data
 *
 * @retval the distance to the closest obstacle or -1 if there are no obstacles
 */
std::tuple<double, const Trajectory*> TrajectoryLibrary::FindFarthestTrajectory(const StereoOctomap &octomap, const BotTrans &body_to_local, double threshold, bot_lcmgl_t* lcmgl) const {

    const Trajectory *farthest_traj = nullptr;

    double traj_closest_dist = -1;

    if (lcmgl != NULL) {
        bot_lcmgl_push_matrix(lcmgl);
    }

    // for each point in each trajectory, find the point that is closest in the octree
    for (int i = 0; i < GetNumberTrajectories(); i++) {

        //std::cout << "Searching trajectory: " << i << std::endl;

        double closest_obstacle_distance = -1;

        int number_of_points = traj_vec_.at(i).GetNumberOfPoints();
        vector<double> point_distances(number_of_points);

        // for each trajectory, look at each point

        // use all availble processors
        #pragma omp parallel for
        for (int j = 0; j < number_of_points; j++) {
            // now we are looking at a single point in a trajectorybot_lcmgl_t *lcmgl

            double transformedPoint[3];

            double this_t = traj_vec_.at(i).GetTimeAtIndex(j);

            traj_vec_.at(i).GetXyzYawTransformedPoint(this_t, body_to_local, transformedPoint);

            //std::cout << "searching at (" << transformedPoint[0] << ", " << transformedPoint[1] << ", " << transformedPoint[2] << ")...";

            point_distances.at(j) = octomap.NearestNeighbor(transformedPoint);

            //std::cout << "distance is: " << distance_to_point << std::endl;

        }

        for (int j = 0; j < number_of_points; j++) {
            double distance_to_point = point_distances.at(j);
            if (distance_to_point >= 0) {
                if (distance_to_point < closest_obstacle_distance || closest_obstacle_distance < 0) {
                    closest_obstacle_distance = distance_to_point;
                }
            }

        }

        if (lcmgl != nullptr) {
            //traj_vector_[i].PlotTransformedTrajectory(lcmgl, body_to_local);
        }

        if (traj_closest_dist == -1 || closest_obstacle_distance > traj_closest_dist) {
            traj_closest_dist = closest_obstacle_distance;
            farthest_traj = &traj_vec_.at(i);

            if (traj_closest_dist > threshold || traj_closest_dist < 0) {
                // we are satisfied with this one, run it!
                if (lcmgl != NULL) {
                    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
                    farthest_traj->Draw(lcmgl, &body_to_local);
                    bot_lcmgl_pop_matrix(lcmgl);
                    bot_lcmgl_switch_buffer(lcmgl);
                }

                return std::tuple<double, const Trajectory*>(traj_closest_dist, farthest_traj);
            }
        }
    }

    if (lcmgl != nullptr) {
        // plot the best trajectory
        if (farthest_traj != NULL) {
            bot_lcmgl_color3f(lcmgl, 1, 0, 0);
            farthest_traj->Draw(lcmgl, &body_to_local);
        }

        bot_lcmgl_pop_matrix(lcmgl);
        bot_lcmgl_switch_buffer(lcmgl);
    }
    return std::tuple<double, const Trajectory*>(traj_closest_dist, farthest_traj);
}


void TrajectoryLibrary::Draw(lcm_t *lcm, const BotTrans *transform) const {
    if (transform == nullptr) {
        BotTrans temp_trans;
        bot_trans_set_identity(&temp_trans);
        transform = &temp_trans;
    }

    bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm, "TrajectoryLibrary");

    for (int i = 0; i < GetNumberTrajectories(); i++) {
        GetTrajectoryByNumber(i)->Draw(lcmgl, transform);
    }
    bot_lcmgl_switch_buffer(lcmgl);

    bot_lcmgl_destroy(lcmgl);
}
