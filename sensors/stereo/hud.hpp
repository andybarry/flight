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
        int frame_number;
        int scale_factor;
        Scalar hud_color;
        
        void PutHudText(Mat hud_img, string str_in, Point text_orgin);
        
        void DrawAirspeed(Mat hud_img);
        void DrawFrameNumber(Mat hud_img);
    
    public:
        Hud();
        
        void SetAirspeed(float airspeed_in_mps) { airspeed = 2.23694*airspeed_in_mps; }
        void SetFrameNumber(int frame_number_in) { frame_number = frame_number_in; }
        
        void DrawHud(InputArray _input_image, OutputArray _output_image);
    
    
    
};

#endif
