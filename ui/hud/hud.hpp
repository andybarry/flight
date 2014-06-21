#ifndef SENSORS_STEREO_HUD_HPP
#define SENSORS_STEREO_HUD_HPP

#include "opencv2/opencv.hpp"
#include <cv.h>
#include <iostream>
#include <string>
#include <bot_core/bot_core.h>

using namespace cv;
using namespace std;

#define PI 3.14159265

class Hud {
    
    private:
        float airspeed_, altitude_, q0_, q1_, q2_, q3_, gps_speed_, gps_heading_, battery_voltage_, x_accel_, y_accel_, z_accel_, throttle_, elevonL_, elevonR_;
        long timestamp_;
        int frame_number_, video_number_;
        int scale_factor_;
        Scalar hud_color_ ;
        int box_line_width_;
        int text_font_;
        double hud_font_scale_;
        double hud_font_scale_small_;
        float text_thickness_;
        float pitch_range_of_lens_;
        int is_autonomous_;
        int clutter_level_;
        
        
        void PutHudText(Mat hud_img, string str_in, Point text_orgin);
        void PutHudTextSmall(Mat hud_img, string str_in, Point text_orgin);
        
        void DrawAirspeed(Mat hud_img);
        void DrawAltitude(Mat hud_img);
        void DrawLadder(Mat hud_img, float value, bool for_airspeed, int major_increment, int minor_increment);
        void DrawFrameNumber(Mat hud_img);
        void DrawGpsSpeed(Mat hud_img);
        void DrawArtificialHorizon(Mat hud_img);
        void DrawCompass(Mat hud_img);
        void DrawBatteryVoltage(Mat hud_img);
        
        void DrawDateTime(Mat hud_img);
        
        
        void DrawElevon(Mat hud_img, float elevon_value, float roll, int center_delta, int width, int position_left, int position_right, bool is_left);
        
        void DrawAutonomous(Mat hud_img);
        
        void DrawCenterMark(Mat hud_img);
        void DrawThrottle(Mat hud_img);

        void DrawAllAccelerationIndicators(Mat hud_img);
        void DrawGraphIndicator(Mat hud_img, int left, int top, string label, int min_value, int max_value, int mark_increment, string plus_format, string minus_format, float value, bool zero_in_center = false, bool reverse_graph_direction = false);
        
        void GetEulerAngles(float *yaw, float *pitch, float *roll);
    
        int GetLadderBoxTop(Mat hud_img) { return hud_img.rows * 0.394; }
        int GetLadderBoxWidth(Mat hud_img) { return hud_img.cols * 0.10; }
        int GetLadderBoxHeight(Mat hud_img) { return hud_img.rows * 0.072; }
        int GetLadderArrowWidth(Mat hud_img) { return hud_img.cols * 0.025; }
        int GetAirspeedLeft(Mat hud_img) { return hud_img.cols * .045; }
        int GetAltitudeLeft(Mat hud_img) { return hud_img.cols * .833; }
    
    public:
        Hud(Scalar hud_color = Scalar(0.45, 0.95, 0.48)); // default is green
        
        void SetAirspeed(float airspeed_in_mps) { airspeed_
            = 2.23694*airspeed_in_mps; }
        void SetAltitude(float altitude_in_meters) { altitude_ = 3.28084*altitude_in_meters; }
        
        void SetGpsSpeed(float gps_speed_in_mps) { gps_speed_ = gps_speed_in_mps * 2.23694; }
        
        void SetOrientation(float q0, float q1, float q2, float q3) {
            q0_ = q0;
            q1_ = q1;
            q2_ = q2;
            q3_ = q3;
        }
        
        void SetAcceleration(float x, float y, float z) {
            x_accel_ = x/9.81;
            y_accel_ = y/9.81;
            z_accel_ = z/9.81 - 1; // minus one since normal G is 1.0
        }
        
        void SetAutonomous(int autonomous) { is_autonomous_ = autonomous; }
        
        void SetServoCommands(float throttle, float elevonL, float elevonR) {
            throttle_ = throttle;
            elevonL_ = elevonL;
            elevonR_ = 100-elevonR; // because the servos are mounted in opposite directions
        }
        
        int GetClutterLevel() { return clutter_level_; }
        void SetClutterLevel(int clutter_level) { clutter_level_ = clutter_level; }
        
        void SetVideoNumber(int video_number) { video_number_ = video_number; }
        
        void SetGpsHeading(float gps_heading) { gps_heading_ = gps_heading; }
        
        void SetBatteryVoltage(float voltage) { battery_voltage_ = voltage; }
        
        void SetPitchRangeOfLens(float pitch_range_of_lens) { pitch_range_of_lens_ = pitch_range_of_lens; }
        float GetPitchRangeOfLens() { return pitch_range_of_lens_; }
        
        void SetFrameNumber(int frame_number_in) { frame_number_ = frame_number_in; }
        
        void SetTimestamp(long timestamp) { timestamp_ = timestamp; }
        
        int GetImageScaling() { return scale_factor_; }
        
        void DrawHud(InputArray _input_image, OutputArray _output_image);
        
        
    
    
    
};

#endif
