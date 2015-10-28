#include "StereoOctomap.hpp"

#define OCTREE_RESOLUTION 128.0f // TODO: set me


StereoOctomap::StereoOctomap(BotFrames *bot_frames) {
    bot_frames_ = bot_frames;

    current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());


    current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    current_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)current_cloud_);

    building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
    building_octree_->setInputCloud(building_cloud_);


    current_octree_timestamp_ = -1;
    building_octree_timestamp_ = -1;

    stereo_calibration_set_ = false;

}

void StereoOctomap::ProcessStereoMessage(const lcmt::stereo *msg) {

    // get transform from global to body frame
    // note that these transforms update live so don't try
    // to cache them
    BotTrans to_open_cv;
    bot_frames_get_trans(bot_frames_, "opencvFrame", "local", &to_open_cv);

    // insert the points into the octree
    InsertPointsIntoOctree(msg, &to_open_cv);

    // zap the old points from the tree
    RemoveOldPoints(msg->timestamp);

}

void StereoOctomap::InsertPointsIntoOctree(const lcmt::stereo *msg, BotTrans *to_open_cv) {
    // apply this matrix to each point
    for (int i = 0; i<msg->number_of_points; i++) {
        double this_point_d[3];
        double trans_point[3];

        this_point_d[0] = msg->x[i];
        this_point_d[1] = msg->y[i];
        this_point_d[2] = msg->z[i];

        // add the position vector
        bot_trans_apply_vec(to_open_cv, this_point_d, trans_point);

        // add point to cloud
        pcl::PointXYZ this_point(trans_point[0], trans_point[1], trans_point[2]);
        current_octree_->addPointToCloud(this_point, current_cloud_);
        building_octree_->addPointToCloud(this_point, building_cloud_);
    }
}


void StereoOctomap::RemoveOldPoints(int64_t last_msg_time) {
    // check timestamps
    if (current_octree_timestamp_ < 0) {
         current_octree_timestamp_ = last_msg_time;
         building_octree_timestamp_ = last_msg_time + OCTREE_LIFE/2; // half a life in the future
    } else if (current_octree_timestamp_ > last_msg_time) {
        // can happen if you're replaying a log and jump back

        // replaying a log, so reinit everything.
        delete current_octree_;
        delete building_octree_;

        // we don't have to delete the clouds since they are shared pointers and will auto-delete

        current_octree_timestamp_ = last_msg_time;
        building_octree_timestamp_ = last_msg_time + OCTREE_LIFE/2;

        current_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        current_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        current_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)current_cloud_);

        building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        building_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)building_cloud_);

        std::cout << std::endl << "swapping octrees because jump back in time" << std::endl;


    } else if (current_octree_timestamp_ + OCTREE_LIFE < last_msg_time && last_msg_time - building_octree_timestamp_ > OCTREE_LIFE/2) {
        std::cout << "swapping octrees (old: " << (current_octree_timestamp_ - last_msg_time) / 1000000.0f << ", new: " << (building_octree_timestamp_ - last_msg_time) / 1000000.0f << ")" << std::endl;
        // swap out trees since this one has expired
        delete current_octree_;
        current_octree_ = building_octree_;

        // we don't have to delete clouds since they are shared pointers and will auto-delete
        current_cloud_ = building_cloud_;

        current_octree_timestamp_ = building_octree_timestamp_;
        building_octree_timestamp_ = last_msg_time;

        building_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);
        building_octree_->setInputCloud((pcl::PointCloud<pcl::PointXYZ>::Ptr)building_cloud_);

        //std::cout << std::endl << "swapping octrees" << std::endl;
    }
}

/**
 * Find the distance to the nearest neighbor of a point
 *
 * @param point the xyz point to search
 *
 * @retval distance to the nearest neighbor or -1 if no points found.
 */
double StereoOctomap::NearestNeighbor(double point[3]) const {

    pcl::PointXYZ search_point;
    search_point.x = point[0];
    search_point.y = point[1];
    search_point.z = point[2];

    // init output parameters
    std::vector<int> point_out_indices(1);
    std::vector<float> k_sqr_distances(1);

    // ensure there is at least one point in the tree
    if (current_octree_->getLeafCount() < 1) {
        // no points in octree
        return -1;
    }

    int num_points_found = current_octree_->nearestKSearch(search_point, 1, point_out_indices, k_sqr_distances);

    if (num_points_found < 1) {
        // no points found
        return -1;
    } else {

        return sqrt(k_sqr_distances.at(0));
    }

}


void StereoOctomap::PrintAllPoints() const {
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = current_cloud_->begin(); it != current_cloud_->end(); it++) {
        std::cout << "(" << it->x << ", " << it->y << ", " << it->z << ")" << std::endl;
    }

}

void StereoOctomap::Draw(lcm_t *lcm) const {
    bot_lcmgl_t *lcmgl = bot_lcmgl_init(lcm, "PointCloud");
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = current_cloud_->begin(); it != current_cloud_->end(); it++) {
        double xyz[3];
        xyz[0] = it->x;
        xyz[1] = it->y;
        xyz[2] = it->z;

        float box_size[3] = { .25, .25, .25 };

        //bot_lcmgl_sphere(lcmgl, xyz, 0.5, 20, 20);
        bot_lcmgl_box(lcmgl, xyz, box_size);
    }
    bot_lcmgl_switch_buffer(lcmgl);

    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
    bot_lcmgl_destroy(lcmgl);
}

/**
 * Publishes the entire map to the camera frame
 * as a stereo message for drawing in the HUD
 */
void StereoOctomap::PublishToHud(lcm_t *lcm) const {
    BotTrans trans;
    bot_frames_get_trans(bot_frames_, "local", "body", &trans);

    lcmt_stereo msg;
    msg.timestamp = GetTimestampNow();
    msg.frame_number = -1;
    msg.video_number = -1;

    int counter = 0;

    // point cloud size
    // TODO: this must be a function call
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = current_cloud_->begin(); it != current_cloud_->end(); it++) {
        counter ++;
    }

    float x[counter], y[counter], z[counter];
    unsigned char grey[counter];

    int i = 0;
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = current_cloud_->begin(); it != current_cloud_->end(); it++) {
        double xyz[3], xyz_camera_frame[3];
        xyz[0] = it->x;
        xyz[1] = it->y;
        xyz[2] = it->z;

        // transform this point into the camera frame
        bot_trans_apply_vec(&trans, xyz, xyz_camera_frame);

        x[i] = xyz_camera_frame[0];
        y[i] = xyz_camera_frame[1];
        z[i] = xyz_camera_frame[2];

        grey[i] = 0;

        i++;
    }

    msg.number_of_points = counter;
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.grey = grey;

    lcmt_stereo_publish(lcm, "octomap-hud", &msg);
}
