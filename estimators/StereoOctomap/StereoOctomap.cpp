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

void StereoOctomap::ProcessStereoMessage(const lcmt_stereo *msg) {

    // get transform from global to body frame
    // note that these transforms update live so don't try
    // to cache them
    BotTrans to_open_cv, body_to_local;
    bot_frames_get_trans(bot_frames_, "opencvFrame", "local", &to_open_cv);
    bot_frames_get_trans(bot_frames_, "body", "local", &body_to_local);

    // insert the points into the octree
    InsertPointsIntoOctree(msg, &to_open_cv, &body_to_local);

    // zap the old points from the tree
    RemoveOldPoints(msg->timestamp);

}

void StereoOctomap::InsertPointsIntoOctree(const lcmt_stereo *msg, BotTrans *to_open_cv, BotTrans *body_to_local) {

    // get the new origin
    //double origin[3];
    //origin[0] = 0;
    //origin[1] = 0;
    //origin[2] = 0;
    //
    //double plane_origin[3];
    //bot_trans_apply_vec(body_to_local, origin, plane_origin);
    //octomath::Vector3 vec_plane_origin(0, 0, 0);


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


    } else if (current_octree_timestamp_ + OCTREE_LIFE < last_msg_time) {
        // swap out trees since this one has expired
        delete current_octree_;

        current_octree_timestamp_ = building_octree_timestamp_;
        building_octree_timestamp_ = last_msg_time;

        current_octree_ = building_octree_;
        building_octree_ = new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(OCTREE_RESOLUTION);

        std::cout << std::endl << "swapping octrees" << std::endl;
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


/*
 * Publishes the octomap to LCM in a format
 * that is readable by fixie_viewer.
 *
 * @param lcm initialized lcm object
 *
 */
/*
void StereoOctomap::PublishOctomap(lcm_t *lcm) {

    // publish octomap to LCM

    octomap_raw_t oc_msg;
    oc_msg.utime = getTimestampNow();

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            oc_msg.transform[i][j] = 0;
        }
        oc_msg.transform[i][i] = 1;
    }

    std::stringstream datastream;
    current_octree_->writeBinaryConst(datastream);
    std::string datastring = datastream.str();
    oc_msg.data = (uint8_t *) datastring.c_str();
    oc_msg.length = datastring.size();

    octomap_raw_t_publish(lcm, "OCTOMAP", &oc_msg);
}
*/

/**
 * Publishes the entire octomap to a lcmt_stereo
 * lcm type.  Useful for comparisons, but likely
 * slow.  Note: transforms points into a local
 * coordinate frame.
 *
 * @param lcm initialized lcm object
 * @param frame_number frame number to publish with the data
 * @param video_number video number to publish with the data
 *
 */
/*
void StereoOctomap::PublishToStereo(lcm_t *lcm, int frame_number, int video_number) {

    if (stereo_calibration_set_ != true) {
        cerr << "Can't call PublishToStereo without setting stereo calibration first." << std::endl;
        return;
    }

    lcmt_stereo_with_xy msg;
    msg.timestamp = getTimestampNow();
    msg.frame_number = frame_number;
    msg.video_number = video_number;


    // get a transform from global to local coordinates
    BotTrans global_to_body;
    bot_frames_get_trans(bot_frames_, "local", "opencvFrame", &global_to_body);


    vector<cv::Point3f> octomap_points;



    GetOctomapPoints(current_octree_, &octomap_points, &global_to_body, true);

    // project them into 2D to put in the message
    vector<Point2f> img_points_list;
    vector<cv::Point3f> octomap_points_transformed;

    if (octomap_points.size() > 0) {

        for (cv::Point3f point : octomap_points) {
            Point3f new_point;
            new_point.x = point.x * stereo_config_.calibrationUnitConversion;
            new_point.y = point.y * stereo_config_.calibrationUnitConversion;
            new_point.z = point.z * stereo_config_.calibrationUnitConversion;

            octomap_points_transformed.push_back(new_point);
        }

        projectPoints(octomap_points_transformed, stereo_calibration_.R1.inv(), Mat::zeros(3, 1, CV_32F), stereo_calibration_.M1, stereo_calibration_.D1, img_points_list);
    }

    vector<float> x_vec(octomap_points.size()), y_vec(octomap_points.size()), z_vec(octomap_points.size());
    vector<int> frame_x_vec(octomap_points.size()), frame_y_vec(octomap_points.size());

    for (unsigned int i = 0; i < octomap_points.size(); i++) {
        Point3f point = octomap_points[i];

        x_vec[i] = point.x;
        y_vec[i] = point.y;
        z_vec[i] = point.z;

        Point2f point2d = img_points_list[i];

        frame_x_vec[i] = round(point2d.x);
        frame_y_vec[i] = round(point2d.y);

    }


    msg.number_of_points = x_vec.size();


    uint8_t grey[msg.number_of_points];

    msg.x = &x_vec[0];
    msg.y = &y_vec[0];
    msg.z = &z_vec[0];
    msg.grey = grey;


    msg.frame_x = &frame_x_vec[0];
    msg.frame_y = &frame_y_vec[0];

    lcmt_stereo_with_xy_publish(lcm, "stereo-octomap", &msg);


}
*/

/**
 * Returns the full list of points in an octree, optionally projected with a transform
 *
 * @param octomap Octree to process
 * @param octomap_points vector<Point3f> that will contain the points
 * @param transform a transform for the points.  For example, to transform all the points to the body
 *  frame, pass: <pre> bot_frames_get_trans(bot_frames, "local", "opencvFrame", &global_to_body); </pre>
 *
 * @param discard_behind if set to true, discards points that have a negative z value
 *  Default: false
 *
 */
/*
void StereoOctomap::GetOctomapPoints(OcTree *octomap, vector<cv::Point3f> *octomap_points, BotTrans *transform, bool discard_behind) {


    // loop through the most likely points on the octomap and plot them on the image
    for(OcTree::leaf_iterator it = octomap->begin_leafs(), end=octomap->end_leafs(); it!= end; ++it) {
        //manipulate node, e.g.:

        // check to see if this is occupied
        if (it->getOccupancy() > octomap->getOccupancyThres()) {

            octomap::point3d this_point = it.getCoordinate();

            // convert this global coordinate into the local coordinate frame
            double this_point_d[3];
            this_point_d[0] = this_point.x();
            this_point_d[1] = this_point.y();
            this_point_d[2] = this_point.z();

            double point_in_transformed_coords[3];


            if (transform != NULL) {
                bot_trans_apply_vec(transform, this_point_d, point_in_transformed_coords);



                if (discard_behind == false || point_in_transformed_coords[2] >= 0) {
                    octomap_points->push_back(cv::Point3f(point_in_transformed_coords[0], point_in_transformed_coords[1], point_in_transformed_coords[2]));
                }

            } else {

                if (discard_behind == false || this_point.z() >= 0) {
                    octomap_points->push_back(cv::Point3f(this_point.x(), this_point.y(), this_point.z()));
                }
            }
        }
    }
}
*/
