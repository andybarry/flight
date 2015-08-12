/**
 * Implements a octree data structure using PCL as a backend
 * for pushbroom stereo data.
 *
 * Supports checking trajectories against the octree to determine nearest neighbor.
 *
 * (C) 2015 Andrew Barry <abarry@csail.mit.edu>
 */

#ifndef STEREO_OCTOMAP_H_
#define STEREO_OCTOMAP_H_


#include <iostream>

#include "opencv2/opencv.hpp"

#include <Eigen/Dense>

#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_core/rotations.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>
#include <lcmtypes/octomap_raw_t.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include "../../LCM/lcmt_stereo_with_xy.h"
#include "../../LCM/lcmt/stereo.hpp"
#include "../../sensors/stereo/opencv-stereo-util.hpp"

#define OCTREE_LIFE 4000000 // in usec

using Eigen::Matrix3d;
using Eigen::Vector3d;


class StereoOctomap {

    public:

        StereoOctomap(BotFrames *bot_frames);

        void ProcessStereoMessage(const lcmt::stereo *msg);

        void PublishOctomap(lcm_t *lcm);
        //void PublishToStereo(lcm_t *lcm, int frame_number, int video_number);

        void SetStereoConfig(OpenCvStereoConfig stereo_config, OpenCvStereoCalibration stereo_calibration) {
            stereo_config_  = stereo_config;
            stereo_calibration_ = stereo_calibration;
            stereo_calibration_set_ = true;
        }

        //static void GetOctomapPoints(OcTree *octomap, vector<cv::Point3f> *octomap_points, BotTrans *transform = NULL, bool discard_behind = false);

        void PrintAllPoints() const;
        void Draw(lcm_t *lcm) const;


        double NearestNeighbor(double point[3]) const;


    private:

        void InsertPointsIntoOctree(const lcmt::stereo *msg, BotTrans *to_open_cv);
        void RemoveOldPoints(int64_t last_msg_time);

        OpenCvStereoCalibration stereo_calibration_;
        OpenCvStereoConfig stereo_config_;
        bool stereo_calibration_set_;

        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *current_octree_;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> *building_octree_;

        int64_t current_octree_timestamp_, building_octree_timestamp_;

        BotFrames *bot_frames_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr building_cloud_;





};

#endif
