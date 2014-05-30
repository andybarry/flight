#ifndef STEREO_FILTER_HPP
#define STEREO_FILTER_HPP


#include <iostream>
#include <math.h>
#include <mutex>

#include "../../LCM/lcmt_stereo.h"

using namespace std;



class StereoFilter {
    
    
    public:
        StereoFilter(float distance_threshold);
        lcmt_stereo* ProcessMessage(const lcmt_stereo *msg);
    
    private:
    
    
        bool FilterSinglePoint(float x, float y, float z);
        float DistanceFunction(float x1, float x2, float y1, float y2, float z1, float z2);
        
        void PrintMsg(const lcmt_stereo *msg, string header = "begin message");
    
    
    
        lcmt_stereo *last_stereo_msg_;
        
        float distance_threshold_;
        
        mutex process_mutex_;
    
    
};

#endif
