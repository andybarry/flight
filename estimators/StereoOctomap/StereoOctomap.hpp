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
#include <octomap/OcTree.h>
#include "../../LCM/lcmt_stereo.h"

#define OCTREE_LIFE 2000000 // in usec

#define STEREO_DIST_TO_METERS_DIVISOR 10 // points come in in cm

using Eigen::Matrix3d;
using Eigen::Vector3d;

using namespace std;
using namespace octomap;



class StereoOctomap {
    
    public:
        
        StereoOctomap(BotFrames *bot_frames);
        
        void ProcessStereoMessage(const lcmt_stereo *msg);
        
        void PublishOctomap(lcm_t *lcm);
        
        
    private:
    
        void InsertPointsIntoOctree(const lcmt_stereo *msg, BotTrans *to_open_cv, BotTrans *body_to_local);
        void RemoveOldPoints(int64_t last_msg_time);
        int64_t getTimestampNow();
    
        OcTree *current_octree_;
        OcTree *building_octree_;
        
        int64_t current_octree_timestamp_, building_octree_timestamp_;
        
        BotFrames *bot_frames_;
        
        
        
        

};

#endif
