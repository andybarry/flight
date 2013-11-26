#include "hud.hpp"

Hud::Hud() {
    airspeed = -101;
    frame_number = 0;
    
    scale_factor = 2;
    hud_color = Scalar(0.45, 0.95, 0.48); // green
}


/**
 * Draws the HUD (Heads Up Display)
 * 
 * @param _input_image image to draw the HUD on
 * 
 * @retval image with the HUD drawn
 */
void Hud::DrawHud(InputArray _input_image, OutputArray _output_image) {
    Mat input_image = _input_image.getMat();
    
    Size output_size = input_image.size()*scale_factor;
    
    
    _output_image.create(output_size, CV_32FC3);
    Mat hud_img = _output_image.getMat();
    
    Mat gray_img;
    input_image.copyTo(gray_img);
    
    Mat gray_img2;
    gray_img.convertTo(gray_img2, CV_32FC3, 1/255.0);
    
    Mat color_img;
    cvtColor(gray_img2, color_img, CV_GRAY2BGR);
    
    resize(color_img, hud_img, output_size);
    
    DrawAirspeed(hud_img);
    DrawFrameNumber(hud_img);
}

void Hud::DrawAirspeed(Mat hud_img)
{
    string airspeed_str;
    if (airspeed > -100) {
        char airspeed_char[100];
        
        sprintf(airspeed_char, "%.1f", airspeed);
        
        airspeed_str = airspeed_char;
    } else {
        airspeed_str = "---";
    }
    
    // draw airspeed ladder box
    
    // figure out the coordinates for the top and bottom on the airspeed box
    int airspeed_box_height = hud_img.rows * 0.072;
    int airspeed_box_width = hud_img.cols * 0.10;
    
    int airspeed_top = hud_img.rows * 0.394;
    int airspeed_left = hud_img.cols * .045;
    
    int arrow_width = hud_img.cols * 0.025;
    
    int box_line_width = 2;
    
    // draw the top line of the box
    line(hud_img, Point(airspeed_left, airspeed_top), Point(airspeed_left + airspeed_box_width, airspeed_top), hud_color, box_line_width);
    
    // draw the left side line
    line(hud_img, Point(airspeed_left, airspeed_top), Point(airspeed_left, airspeed_top + airspeed_box_height), hud_color, box_line_width);
    
    // draw the bottom line of the box
    line(hud_img, Point(airspeed_left, airspeed_top + airspeed_box_height), Point(airspeed_left + airspeed_box_width, airspeed_top + airspeed_box_height), hud_color, box_line_width);
    
    // draw the top of the arrow
    line(hud_img, Point(airspeed_left + airspeed_box_width, airspeed_top), Point(airspeed_left + airspeed_box_width + arrow_width, airspeed_top + airspeed_box_height / 2), hud_color, box_line_width);
    
    // draw the bottom of the arrow
    line(hud_img, Point(airspeed_left + airspeed_box_width, airspeed_top + airspeed_box_height), Point(airspeed_left + airspeed_box_width + arrow_width, airspeed_top + airspeed_box_height / 2), hud_color, box_line_width);
    
    
    // draw the airspeed numbers on the HUD
    
    // get the size of the text string
    int baseline = 0;
    Size text_size = getTextSize(airspeed_str, FONT_HERSHEY_DUPLEX, 0.9, 1, &baseline);
    
    // right align the numbers in the box
    Point text_orgin(airspeed_left + airspeed_box_width - text_size.width,
        airspeed_top + airspeed_box_height - baseline);
    
    //putText(hud_img, airspeed_str, Point(airspeed_left + box_line_width + text_gap, airspeed_top - text_gap - box_line_width + airspeed_box_height), FONT_HERSHEY_DUPLEX, 0.9, hud_color);
    
    PutHudText(hud_img, airspeed_str, text_orgin);
}

void Hud::DrawFrameNumber(Mat hud_img) {
    // draw the frame number in the lower right
    
    char frame_char[100];
    
    sprintf(frame_char, "F%d", frame_number);
    
    string frame_str = frame_char;
    
    Point text_origin(0.83 * hud_img.cols, 0.876*hud_img.rows);
    
    PutHudText(hud_img, frame_str, text_origin);
}

void Hud::PutHudText(Mat hud_img, string str_in, Point text_orgin) {
    double hud_font_scale = 0.9;
    
    putText(hud_img, str_in, text_orgin, FONT_HERSHEY_DUPLEX, hud_font_scale, hud_color);
}
