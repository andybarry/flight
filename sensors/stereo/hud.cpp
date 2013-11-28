#include "hud.hpp"

Hud::Hud() {
    airspeed_ = -10001;
    altitude_ = -10001;
    frame_number_ = 0;
    
    scale_factor_ = 2;
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
    
    Size output_size = input_image.size()*scale_factor_;
    
    
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
    DrawLadder(hud_img, airspeed_, true, 10, 2);
    
    DrawAltitude(hud_img);
    DrawLadder(hud_img, altitude_, false, 20, 4);
    
    DrawFrameNumber(hud_img);
}

void Hud::DrawAirspeed(Mat hud_img) {
    string airspeed_str;
    if (airspeed_ > -10000) {
        char airspeed_char[100];
        
        sprintf(airspeed_char, "%.0f", airspeed_);
        
        airspeed_str = airspeed_char;
    } else {
        airspeed_str = "---";
    }
    
    // draw airspeed ladder box
    
    // figure out the coordinates for the top and bottom on the airspeed box
    int airspeed_box_height = GetLadderBoxHeight(hud_img);
    int airspeed_box_width = GetLadderBoxWidth(hud_img);
    
    int airspeed_top = GetLadderBoxTop(hud_img);
    int airspeed_left = GetAirspeedLeft(hud_img);
    
    int arrow_width = GetLadderArrowWidth(hud_img);
    
    // draw the top line of the box
    line(hud_img, Point(airspeed_left, airspeed_top), Point(airspeed_left + airspeed_box_width, airspeed_top), hud_color_, box_line_width_);
    
    // draw the left side line
    line(hud_img, Point(airspeed_left, airspeed_top), Point(airspeed_left, airspeed_top + airspeed_box_height), hud_color_, box_line_width_);
    
    // draw the bottom line of the box
    line(hud_img, Point(airspeed_left, airspeed_top + airspeed_box_height), Point(airspeed_left + airspeed_box_width, airspeed_top + airspeed_box_height), hud_color_, box_line_width_);
    
    // draw the top of the arrow
    line(hud_img, Point(airspeed_left + airspeed_box_width, airspeed_top), Point(airspeed_left + airspeed_box_width + arrow_width, airspeed_top + airspeed_box_height / 2), hud_color_, box_line_width_);
    
    // draw the bottom of the arrow
    line(hud_img, Point(airspeed_left + airspeed_box_width, airspeed_top + airspeed_box_height), Point(airspeed_left + airspeed_box_width + arrow_width, airspeed_top + airspeed_box_height / 2), hud_color_, box_line_width_);
    
    
    // draw the airspeed numbers on the HUD
    
    // get the size of the text string
    int baseline = 0;
    Size text_size = getTextSize(airspeed_str, text_font_, hud_font_scale_, text_thickness_, &baseline);
    
    // right align the numbers in the box
    Point text_orgin(airspeed_left + airspeed_box_width - text_size.width,
        airspeed_top + airspeed_box_height - baseline);
    
    //putText(hud_img, airspeed_str, Point(airspeed_left + box_line_width + text_gap, airspeed_top - text_gap - box_line_width + airspeed_box_height), FONT_HERSHEY_DUPLEX, 0.9, hud_color);
    
    PutHudText(hud_img, airspeed_str, text_orgin);
}

void Hud::DrawAltitude(Mat hud_img) {
    
    // convert into a reasonable string
    string altitude_str;
    if (altitude_ > -10000) {
        char altitude_char[100];
        
        sprintf(altitude_char, "%.0f", altitude_);
        
        altitude_str = altitude_char;
    } else {
        altitude_str = "---";
    }
    
    int top = GetLadderBoxTop(hud_img);
    
    int left = GetAltitudeLeft(hud_img);
    
    int width = GetLadderBoxWidth(hud_img);
    int height = GetLadderBoxHeight(hud_img);
    
    // draw the altitude box
    
    // draw the top line of the box
    line(hud_img, Point(left, top), Point(left + width, top), hud_color_, box_line_width_);
    
    // draw the right side line
    line(hud_img, Point(left + width, top), Point(left + width, top + height), hud_color_, box_line_width_);
    
    // draw the bottom line of the box
    line(hud_img, Point(left, top + height), Point(left + width, top + height), hud_color_, box_line_width_);
    
    // draw the top of the arrow
    int arrow_width = GetLadderArrowWidth(hud_img);
    line(hud_img, Point(left - arrow_width, top + height/2), Point(left, top), hud_color_, box_line_width_);
    
    // draw the bottom of the arrow
    line(hud_img, Point(left - arrow_width, top + height/2), Point(left, top + height), hud_color_, box_line_width_);
    
    // get the size of the text string
    int baseline = 0;
    Size text_size = getTextSize(altitude_str, text_font_, hud_font_scale_, text_thickness_, &baseline);
    
    // left align the numbers in the box
    Point text_orgin(left + width - text_size.width - 5, top + height - baseline);
    
    // now draw the text
    PutHudText(hud_img, altitude_str, text_orgin);
}

/**
 * Draw the lines for the airspeed and altitude ladders
 * 
 * @param hud_img image to draw on
 * @param value value of the ladder being drawn
 * @param for_airspeed true if the ladder is for the airspeed, false if it is
 *      for altitude
 * @param major_increment amount between labels / larger lines
 * 
 */
void Hud::DrawLadder(Mat hud_img, float value, bool for_airspeed, int major_increment, int minor_increment) {
    
    
    int left;
    int offset = 10;
    int vertical_line_gap = 0.021 * hud_img.rows;
    int ladder_width = 0.0186 * hud_img.cols;
    int major_line_extra_width = 0.01 * hud_img.cols;
    
    // ladder should start a few pixels to the right (or left) of the box/arrow
    if (for_airspeed) {
        left = GetAirspeedLeft(hud_img) + offset + GetLadderBoxWidth(hud_img) + GetLadderArrowWidth(hud_img);
    } else {
        left = GetAltitudeLeft(hud_img) - GetLadderArrowWidth(hud_img) - offset - ladder_width - major_line_extra_width;
    }
    
    int text_gap = 4;
    
    int top = 0.221 * hud_img.rows;
    int bottom = 0.652 * hud_img.rows;
    
    int number_of_lines = (bottom - top) / vertical_line_gap;
    int total_number_span = number_of_lines * minor_increment;
    
    // figure out what the top pixel would correspond to
    float value_per_px = (bottom - top) / total_number_span;
    
    float top_px_value = number_of_lines / 2 * minor_increment + value;
    
    // now we know what float the top pixel corresponds to
    // now we want to compute where we're going to draw lines
    // that means we need to know the first line to draw at the top
    // which means we need to round down to the nearest minor increment
    // from the top_px_value
    
    int diff_from_minor_increment = int(round(top_px_value)) % minor_increment;
    int top_line_value = int(round(top_px_value)) - diff_from_minor_increment;
    int top_line_position = top - (float)diff_from_minor_increment / value_per_px;

    int extra_height;
    
    int is_minor;
    
    for (int i = 0; i < number_of_lines; i++) {

        int this_top = top_line_position + i * vertical_line_gap;
        int this_value = top_line_value - i * minor_increment;
        
        if (this_value % major_increment == 0) {
            is_minor = 0;
            extra_height = 1;
        } else {
            is_minor = 1;
            extra_height = 0;
        }

        // draw the line
        if (for_airspeed) {
            line(hud_img, Point(left + major_line_extra_width * is_minor, this_top), Point(left + ladder_width, this_top), hud_color_, box_line_width_ + extra_height);
            
        } else {
            // todo
            line(hud_img, Point(left, this_top), Point(left + ladder_width + major_line_extra_width * (!is_minor), this_top), hud_color_, box_line_width_ + extra_height);
        }
        
        // draw a label if this is a major
        if (!is_minor && this_value > -9000) {
            
            string this_label = to_string(this_value);
            
            int baseline = 0;
            Size text_size = getTextSize(this_label, text_font_, hud_font_scale_, text_thickness_, &baseline);
            
            int text_center = this_top + text_size.height / 2;
            
            // don't draw if we're close to the centerline
            if (abs(text_center - (GetLadderBoxTop(hud_img) + GetLadderBoxHeight(hud_img) / 2)) > text_size.height*2) {
                Point text_origin;
                if (for_airspeed) {
                    text_origin = Point(left - text_size.width - text_gap, this_top + text_size.height/2);
                } else {
                    text_origin = Point(left + text_gap + ladder_width + major_line_extra_width, this_top + text_size.height/2);
                }
                
                PutHudText(hud_img, this_label, text_origin);
            }            
        }
    }
    
    
    //int center = GetLadderBoxTop(hud_img) + GetLadderBoxHeight(hud_img)/2;
    
    // loop from the center down, adding lines as we go
    
    
    
    
}


void Hud::DrawFrameNumber(Mat hud_img) {
    // draw the frame number in the lower right
    
    char frame_char[100];
    
    sprintf(frame_char, "F%d", frame_number_);
    
    string frame_str = frame_char;
    
    Point text_origin(0.83 * hud_img.cols, 0.876*hud_img.rows);
    
    PutHudText(hud_img, frame_str, text_origin);
}

void Hud::PutHudText(Mat hud_img, string str_in, Point text_orgin) {
    putText(hud_img, str_in, text_orgin, text_font_, hud_font_scale_, hud_color_);
}
