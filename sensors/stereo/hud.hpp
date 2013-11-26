#ifndef SENSORS_STEREO_HUD_HPP
#define SENSORS_STEREO_HUD_HPP

#include "opencv2/opencv.hpp"
#include <cv.h>
#include <iostream>

using namespace cv;
using namespace std;

class Hud {
    
    private:
        float airspeed;
    
    public:
        Hud();
        
        void SetAirspeed(float airspeed_in) { airspeed = airspeed_in; }
        
        void DrawHud(InputArray _input_image, OutputArray _output_image);
    
    
    
};

#endif
