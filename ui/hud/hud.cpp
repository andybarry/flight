#include "hud.hpp"

Hud::Hud(Scalar hud_color) {
    scale_factor_ = 2;
    hud_color_ = hud_color;
    box_line_width_ = 2;
    text_font_ = FONT_HERSHEY_DUPLEX;
    hud_font_scale_ = 0.45 * scale_factor_;
    hud_font_scale_small_ = 0.3 * scale_factor_;
    text_thickness_ = 1;
    pitch_range_of_lens_ = 108.2; // measured for the calibrated and rectified image

    airspeed_ = -10001;
    altitude_ = -10001;
    gps_speed_= -10001;
    battery_voltage_ = -10001;
    throttle_ = 0;
    elevonL_ = 50;
    elevonR_ = 50;
    x_accel_ = 0;
    y_accel_ = 0;
    z_accel_ = 0;
    gps_heading_ = 0;
    frame_number_ = 0;
    video_number_ = -1;
    q0_ = 0;
    q1_ = 0;
    q2_ = 0;
    q3_ = 0;
    is_autonomous_ = 0;
    clutter_level_ = 5;
}


/**
 * Draws the HUD (Heads Up Display)
 *
 * @param _input_image image to draw the HUD on
 * @param _output_image image that is returned and contains the HUD
 *
 */
void Hud::DrawHud(InputArray _input_image, OutputArray _output_image) {

    Mat input_image = _input_image.getMat();

    Mat hud_img;

    // check to see if the input is a black and white image
    // or a color image

    // if it is color, we're probably writing on top of an existing
    // HUD for comparison, so don't convert color

    Size output_size = input_image.size()*scale_factor_;

    _output_image.create(output_size, CV_32FC3);
    hud_img = _output_image.getMat();

    if (input_image.type() == CV_8UC1) {

        Mat gray_img;
        input_image.copyTo(gray_img);

        Mat gray_img2;
        gray_img.convertTo(gray_img2, CV_32FC3, 1/255.0);

        Mat color_img;
        cvtColor(gray_img2, color_img, CV_GRAY2BGR);

        resize(color_img, hud_img, output_size);
    } else {
        resize(input_image, hud_img, output_size);
    }

    if (clutter_level_ == 99) {
        DrawArtificialHorizon(hud_img);

    } else {

        if (clutter_level_ > 0) {
            DrawAirspeed(hud_img);
            DrawLadder(hud_img, airspeed_, true, 10, 2);

            DrawAltitude(hud_img);
            DrawLadder(hud_img, altitude_, false, 20, 4);

            DrawCenterMark(hud_img);

        }

        if (clutter_level_ > 1) {
            DrawGpsSpeed(hud_img);
            DrawAutonomous(hud_img);
            DrawFrameNumber(hud_img);
            DrawBatteryVoltage(hud_img);
            DrawDateTime(hud_img);

        }

        if (clutter_level_ > 2) {
            DrawArtificialHorizon(hud_img);
            DrawThrottle(hud_img);
        }

        if (clutter_level_ > 3) {
            DrawAllAccelerationIndicators(hud_img);
        }


        if (clutter_level_ > 4) {
            DrawCompass(hud_img);
        }
    }




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
    int gps_gap = 10;
    int gps_triangle_size = 10;
    int vertical_line_gap = 0.021 * hud_img.rows;
    int ladder_width = 0.0186 * hud_img.cols;
    int major_line_extra_width = 0.01 * hud_img.cols;

    // ladder should start a few pixels to the right (or left) of the box/arrow
    if (for_airspeed) {
        left = GetAirspeedLeft(hud_img) + offset + GetLadderBoxWidth(hud_img) + GetLadderArrowWidth(hud_img);
    } else {
        left = GetAltitudeLeft(hud_img) - GetLadderArrowWidth(hud_img) - offset - ladder_width;
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
            line(hud_img, Point(left, this_top), Point(left + ladder_width - major_line_extra_width * is_minor, this_top), hud_color_, box_line_width_ + extra_height);
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

        // if this is the airspeed, we also want to draw an arrow showing where
        // gps speed falls
        if (for_airspeed && gps_speed_ > -10000) {

            int gps_left = left + ladder_width + gps_gap;
            int gps_center = (top_px_value - gps_speed_) * value_per_px + top;

            // check to see if GPS speed is offscale high
            if (gps_center < top) {

                // off scale high

                // draw and upwards facing arrow at the top
                line(hud_img, Point(gps_left, top + gps_triangle_size/2), Point(gps_left + gps_triangle_size/2, top), hud_color_, box_line_width_);

                line(hud_img, Point(gps_left + gps_triangle_size, top + gps_triangle_size/2), Point(gps_left + gps_triangle_size/2, top), hud_color_, box_line_width_);

            } else if (gps_center > bottom) {

                // gps is offscale low

                // draw a downwards facing arrow at the bottom
                line(hud_img, Point(gps_left, bottom - vertical_line_gap - gps_triangle_size/2), Point(gps_left + gps_triangle_size/2, bottom - vertical_line_gap), hud_color_, box_line_width_);

                line(hud_img, Point(gps_left + gps_triangle_size, bottom - vertical_line_gap - gps_triangle_size/2), Point(gps_left + gps_triangle_size/2, bottom - vertical_line_gap), hud_color_, box_line_width_);


            } else {
                // gps is on scale

                line(hud_img, Point(gps_left, gps_center), Point(gps_left + gps_triangle_size, gps_center + gps_triangle_size/2), hud_color_, box_line_width_);

                line(hud_img, Point(gps_left, gps_center), Point(gps_left + gps_triangle_size, gps_center - gps_triangle_size/2), hud_color_, box_line_width_);
            }
        }
    }
}

void Hud::DrawArtificialHorizon(Mat hud_img) {

    float yaw, pitch, roll;

    GetEulerAngles(&yaw, &pitch, &roll);

    int width = 150;
    int center_gap = 50;
    int tip_length = 10;

    int elevon_pos_max = width * 7.0/8.0;
    int elevon_pos_min = width * 5.0/8.0;


    float px_per_deg = hud_img.rows / pitch_range_of_lens_;
    int center_height = hud_img.rows/2 - pitch*px_per_deg;

    int center_delta = center_height - hud_img.rows/2;

    int left = hud_img.cols/2 - (width * sin(PI/180.0 * (roll + 90)));
    int top = hud_img.rows/2 - (width * cos(PI/180.0 * (roll + 90))) + center_delta;

    int bottom = hud_img.rows/2 - (center_gap * cos(PI/180.0 * (roll + 90))) + center_delta;
    int right = hud_img.cols/2 - (center_gap * sin(PI/180.0 * (roll + 90)));

    // draw the left half of the line
    line(hud_img, Point(left, top), Point(right, bottom), hud_color_, box_line_width_);

    // draw the 90 degree tips on the lines
    int angle_left = left + sin(PI/180.0*(roll+180)) * tip_length;
    int angle_top = top + cos(PI/180.0*(roll+180)) * tip_length;

    line(hud_img, Point(angle_left, angle_top), Point(left, top), hud_color_, box_line_width_);

    // draw the left elevon box
    if (elevonL_ != 0 || elevonR_ != 0) {
        // draw the left side elevon
        DrawElevon(hud_img, elevonL_, roll, center_delta, width, elevon_pos_max, elevon_pos_min, true);

        DrawElevon(hud_img, elevonR_, roll, center_delta, width, elevon_pos_max, elevon_pos_min, false);
    }

    // draw the right half of the line now
    left = hud_img.cols/2 + (center_gap * sin(PI/180.0 * (roll + 90)));
    top = hud_img.rows/2 + (center_gap * cos(PI/180.0 * (roll + 90))) + center_delta;

    bottom = hud_img.rows/2 + (width * cos(PI/180.0 * (roll + 90))) + center_delta;
    right = hud_img.cols/2 + (width * sin(PI/180.0 * (roll + 90)));

    line(hud_img, Point(left, top), Point(right, bottom), hud_color_, box_line_width_);

    // draw the right have tip
    angle_left = right + sin(PI/180.0*(roll+180)) * tip_length;
    angle_top = bottom + cos(PI/180.0*(roll+180)) * tip_length;

    line(hud_img, Point(angle_left, angle_top), Point(right, bottom), hud_color_, box_line_width_);

    #if 0
    int text_gap = 10;
    // draw the pitch label
    string pitch_str;
    char pitch_char[100];

    sprintf(pitch_char, "%.0f", pitch);

    pitch_str = pitch_char;

    int baseline = 0;
    Size text_size = getTextSize(pitch_str, text_font_, hud_font_scale_small_, text_thickness_, &baseline);

    PutHudTextSmall(hud_img, pitch_str, Point(right + text_gap, bottom + text_size.height/3));

    // draw the roll label

    string roll_str;
    char roll_char[100];

    sprintf(roll_char, "%.0f", roll);

    roll_str = roll_char;

    baseline = 0;
    text_size = getTextSize(roll_str, text_font_, hud_font_scale_, text_thickness_, &baseline);

    PutHudText(hud_img, roll_str, Point(hud_img.cols/2 - text_size.width/2, center_height - baseline));

    #endif

}

void Hud::DrawElevon(Mat hud_img, float elevon_value, float roll, int center_delta, int width, int position_left, int position_right, bool is_left) {

    int max_elevon_size = 100;
    float elevon_overflow = 0;

    if (elevon_value > max_elevon_size) {
        elevon_overflow = elevon_value - max_elevon_size;
        elevon_value = max_elevon_size;
    }

    if (elevon_value < 0) {
        elevon_overflow = elevon_value;
        elevon_value = 0;
    }

    int top_line_left = 0, top_line_top = 0, top_line_right = 0, top_line_bottom = 0;

    int sign_change = 1;
    if (is_left == false) {
        sign_change = -1;
    }

    int position;
    for (int i = 0; i < 2; i++) {
        if (i == 0) {
            position = position_left;
        } else {
            position = position_right;
        }

        // point on the artificial horizon (x and y coordinates)
        int elevon_left_side_left = hud_img.cols/2 - (position * sin(PI/180.0 * (roll + 90))) * sign_change;
        int elevon_left_side_top = hud_img.rows/2 - (position * cos(PI/180.0 * (roll + 90))) * sign_change + center_delta;

        // points offset from the artificial horizon that depend on elevon_value (x and y coordinates)
        int elevon_left_side_right = elevon_left_side_left + sin(PI/180.0*(roll+180)) * max_elevon_size * (elevon_value-50)/100.0;
        int elevon_left_side_bottom = elevon_left_side_top + cos(PI/180.0*(roll+180)) * max_elevon_size * (elevon_value-50)/100.0;

        if (i == 0) {
            top_line_left = elevon_left_side_right;
            top_line_top = elevon_left_side_bottom;
        } else {
            top_line_right = elevon_left_side_right;
            top_line_bottom = elevon_left_side_bottom;
        }

        line(hud_img, Point(elevon_left_side_left, elevon_left_side_top), Point(elevon_left_side_right, elevon_left_side_bottom), hud_color_, box_line_width_);
    }

    // now draw the top
    line(hud_img, Point(top_line_left, top_line_top), Point(top_line_right, top_line_bottom), hud_color_, box_line_width_);


    // deal with overflow if any
    if (elevon_overflow != 0) {

        int overflow_top_line_left = 0, overflow_top_line_top = 0, overflow_top_line_right = 0, overflow_top_line_bottom = 0;

        for (int i = 0; i < 2; i++) {

            int elevon_overflow_left;
            int elevon_overflow_top;

            if (i == 0) {
                elevon_overflow_left = top_line_left;
                elevon_overflow_top = top_line_top;
            } else {
                elevon_overflow_left = top_line_right;
                elevon_overflow_top = top_line_bottom;
            }

            // now extend the overflow bar
            int elevon_overflow_right = elevon_overflow_left + sin(PI/180.0*(roll+180)) * max_elevon_size * (elevon_overflow)/100.0;
            int elevon_overflow_bottom = elevon_overflow_top + cos(PI/180.0*(roll+180)) * max_elevon_size * (elevon_overflow)/100.0;

            // draw the line
            line(hud_img, Point(elevon_overflow_left, elevon_overflow_top), Point(elevon_overflow_right, elevon_overflow_bottom), hud_color_, box_line_width_);

            if (i == 0) {
                overflow_top_line_left = elevon_overflow_right;
                overflow_top_line_top = elevon_overflow_bottom;
            } else {
                overflow_top_line_right = elevon_overflow_right;
                overflow_top_line_bottom = elevon_overflow_bottom;
            }
        }

        // draw the line on top of the overflow bar
        line(hud_img, Point(overflow_top_line_left, overflow_top_line_top), Point(overflow_top_line_right, overflow_top_line_bottom), hud_color_, box_line_width_);

        // now we have drawn the outside box -- draw slashes to indicate that this
        // command isn't going to happen

        // compute the number of slashes needed
        int slash_spacing = 200;

        int rise = overflow_top_line_bottom - overflow_top_line_top;
        int run = overflow_top_line_right - overflow_top_line_left;
        float cross_slash_slope = float(overflow_top_line_top - top_line_bottom) / float(overflow_top_line_left - top_line_right);

        int current_x, current_y;

        float delta_x = sqrt( slash_spacing / (1 + cross_slash_slope*cross_slash_slope) );
        float delta_y = cross_slash_slope * delta_x;

        cout << delta_x << ", " << delta_y << endl;

        line(hud_img, Point(overflow_top_line_left, overflow_top_line_top), Point(top_line_right, top_line_bottom), hud_color_, box_line_width_);

        float slope = 1;

        for (int i = 0; i < 10; i++) {

            current_x = overflow_top_line_left + i*delta_x;
            current_y = overflow_top_line_top + i*delta_y;

            // now the current x and y are the points along a line that is the hypotenuse of the box
            // running the in opposite direction of the slashes

            // if we extend this point with a constant slope to both sides, we'll have the point along
            // the sides to draw a line

            circle(hud_img, Point(current_x, current_y), 10, hud_color_, 1);

            int y = slope * ( top_line_right - current_x) + current_y;

            circle(hud_img, Point(top_line_right, y), 10, hud_color_, 1);

        }

    }


}

void Hud::GetEulerAngles(float *yaw, float *pitch, float *roll) {

    double q[4];
    q[0] = q0_;
    q[1] = q1_;
    q[2] = q2_;
    q[3] = q3_;

    double rpy[3];

    bot_quat_to_roll_pitch_yaw(q, rpy);

    *yaw = rpy[2] * 180 / PI;
    *pitch = rpy[1] * 180 / PI;
    *roll = rpy[0] * 180 / PI;

    /*
    *roll = 180/PI * atan2(2*(q0_ * q1_ + q2_ * q3_), ( 1 - 2*(q1_*q1_ + q2_ * q2_) ) );

    *pitch = -180/PI * asin(2 * (q0_ * q2_ - q3_ * q1_ ) );

    *yaw = 180/PI * atan2( 2* (q0_ * q3_ + q1_ * q2_), ( 1 - 2*(q2_*q2_ + q3_ * q3_) ) );
    */
    //printf("q0: %f, q1: %f, q2: %f, q3: %f\n", q[0], q[1], q[2], q[3]);
    //printf("roll %f / %f pitch %f / %f, yaw %f / %f\n", *roll, rpy[0] * 180/PI, *pitch, rpy[1] * 180/PI, *yaw, rpy[2] * 180/PI);
}

void Hud::DrawFrameNumber(Mat hud_img) {
    // draw the frame number in the lower right

    char frame_char[100];

    if (video_number_ >= 0) {
        sprintf(frame_char, "F%02d.%d", video_number_, frame_number_);
    } else {
        sprintf(frame_char, "F%d", frame_number_);
    }

    string frame_str = frame_char;

    Point text_origin(0.76 * hud_img.cols, 0.866*hud_img.rows);

    PutHudText(hud_img, frame_str, text_origin);
}

void Hud::PutHudText(Mat hud_img, string str_in, Point text_orgin) {
    putText(hud_img, str_in, text_orgin, text_font_, hud_font_scale_, hud_color_);
}

void Hud::PutHudTextSmall(Mat hud_img, string str_in, Point text_orgin) {
    putText(hud_img, str_in, text_orgin, text_font_, hud_font_scale_small_, hud_color_);
}

void Hud::DrawGpsSpeed(Mat hud_img) {
    char gps_char[100];

    sprintf(gps_char, "GS %.1f", gps_speed_);


    string gps_str;

    if (gps_speed_ > -10000) {
        gps_str = gps_char;
    } else {
        gps_str = "GS ---";
    }

    Point text_origin(0.1 * hud_img.cols, 0.75*hud_img.rows);

    PutHudText(hud_img, gps_str, text_origin);

}

void Hud::DrawBatteryVoltage(Mat hud_img) {
    char battery_char[100];

    sprintf(battery_char, "%.1fV", battery_voltage_);


    string battery_str;

    if (battery_voltage_ > -10000) {
        battery_str = battery_char;
    } else {
        battery_str = "--- V";
    }

    Point text_origin(0.76 * hud_img.cols, 0.93*hud_img.rows);

    PutHudText(hud_img, battery_str, text_origin);

}

void Hud::DrawCompass(Mat hud_img) {
    float yaw, pitch, roll;

    GetEulerAngles(&yaw, &pitch, &roll);

    // yaw is our compass angle
    int compass_center_width = hud_img.cols/2;
    int compass_center_height = hud_img.rows * 1.10;
    int compass_radius = 0.133 * hud_img.cols;
    int line_size = 10;
    int text_gap = 5;
    int arrow_gap = 10;
    int gps_arrow_size = 5;

    int compass_increment = 10; // in degrees

    // add labels
    int baseline = 0;
    Size text_size = getTextSize("N", text_font_, hud_font_scale_small_, text_thickness_, &baseline);

    for (int degree = 0; degree < 360; degree += compass_increment) {
        // draw the compass lines

        int this_degree = degree + yaw;

        int extra_line_width = 0;
        int extra_line_length = 0;

        if (degree % 90 == 0) {
            extra_line_width = 1;
            extra_line_length = 3;
        }

        int top = sin(PI/180.0 * this_degree) * compass_radius + compass_center_height;
        int left = cos(PI/180.0 * this_degree) * compass_radius + compass_center_width;

        int mid_top = sin(PI/180.0 * this_degree) * (compass_radius-line_size-extra_line_length) + compass_center_height;
        int mid_left = cos(PI/180.0 * this_degree) * (compass_radius-line_size-extra_line_length) + compass_center_width;


        switch (degree) {
            case 90:
                PutHudTextSmall(hud_img, "N", Point(mid_left - text_size.width/2, mid_top + text_size.height + text_gap));
                break;

            case 180:
                PutHudTextSmall(hud_img, "E", Point(mid_left - text_size.width/2, mid_top + text_size.height + text_gap));
                break;

            case 270:
                PutHudTextSmall(hud_img, "S", Point(mid_left - text_size.width/2, mid_top + text_size.height + text_gap));
                break;

            case 0:
                PutHudTextSmall(hud_img, "W", Point(mid_left - text_size.width/2, mid_top + text_size.height + text_gap));
                break;
        }

        line(hud_img, Point(left, top), Point(mid_left, mid_top), hud_color_, box_line_width_ + extra_line_width);

    }

    // draw the arrow on the top pointing down
    //line(hud_img, Point(compass_center_width - line_size/2, compass_center_height - compass_radius - line_size/2 - arrow_gap), Point(compass_center_width, compass_center_height - compass_radius - arrow_gap), hud_color_, box_line_width_);

    //line(hud_img, Point(compass_center_width + line_size/2, compass_center_height - compass_radius - line_size/2 - arrow_gap), Point(compass_center_width, compass_center_height - compass_radius - arrow_gap), hud_color_, box_line_width_);

    line(hud_img, Point(compass_center_width, compass_center_height - compass_radius - line_size - arrow_gap), Point(compass_center_width, compass_center_height - compass_radius - arrow_gap), hud_color_, box_line_width_);

    // print the actual angle on the screen

    char heading_char[100];

    sprintf(heading_char, "%.0f", 180-yaw);

    string heading_str = heading_char;

    baseline = 0;
    text_size = getTextSize(heading_str, text_font_, hud_font_scale_small_, text_thickness_, &baseline);

    PutHudTextSmall(hud_img, heading_str, Point(compass_center_width - text_size.width/2, compass_center_height - compass_radius - line_size - arrow_gap - baseline));

    // draw the GPS heading as an arrow on the compass
    float this_heading = -90 + (gps_heading_ - (180-yaw));
    int gps_top = sin(PI/180.0 * this_heading) * (compass_radius + arrow_gap) + compass_center_height;
    int gps_left = cos(PI/180.0 * this_heading) * (compass_radius + arrow_gap) + compass_center_width;

    int gps_mid_top = sin(PI/180.0 * (this_heading+4)) * (compass_radius + arrow_gap + gps_arrow_size) + compass_center_height;
    int gps_mid_left = cos(PI/180.0 * (this_heading+4)) * (compass_radius + arrow_gap + gps_arrow_size) + compass_center_width;

    int gps_mid_top2 = sin(PI/180.0 * (this_heading-4)) * (compass_radius + arrow_gap + gps_arrow_size) + compass_center_height;
    int gps_mid_left2 = cos(PI/180.0 * (this_heading-4)) * (compass_radius + arrow_gap + gps_arrow_size) + compass_center_width;

    // draw a little arrow showing where GPS heading is
    line(hud_img, Point(gps_left, gps_top), Point(gps_mid_left, gps_mid_top), hud_color_, box_line_width_);

    line(hud_img, Point(gps_left, gps_top), Point(gps_mid_left2, gps_mid_top2), hud_color_, box_line_width_);


    //line(hud_img, Point(gps_left, gps_top), Point(gps_mid_left, gps_mid_top), hud_color_, box_line_width_ + 10);
}

void Hud::DrawAllAccelerationIndicators(Mat hud_img) {

    int x = 0.83 * hud_img.rows;
    int y = 0.89 * hud_img.rows;
    int z = 0.95 * hud_img.rows;

    int left = 0.05 * hud_img.cols;

    int max_value = 4;
    int min_value = -4;
    int mark_increment = 1;

    DrawGraphIndicator(hud_img, left, x, "x", min_value, max_value, mark_increment, "+%.1fG", "-%.1fG", x_accel_, true, true);
    DrawGraphIndicator(hud_img, left, y, "y", min_value, max_value, mark_increment, "+%.1fG", "-%.1fG", y_accel_, true, true);
    DrawGraphIndicator(hud_img, left, z, "z", min_value, max_value, mark_increment, "+%.1fG", "-%.1fG", z_accel_, true, true);
}

void Hud::DrawGraphIndicator(Mat hud_img, int left, int top, string label, int min_value, int max_value, int mark_increment, string plus_format, string minus_format, float value, bool zero_in_center, bool reverse_graph_direction) {
    // draw graphs for x, y, and z acceleration


    int width = 0.19 * hud_img.cols;

    int height = 0.01 * hud_img.cols;

    int line_width = 1;
    int center_line_height_extra = 2;
    int arrow_width = 10;
    int arrow_gap = 3;
    int arrow_height = 10;
    int text_gap = 10;

    // draw the bottom line of the graph
    line(hud_img, Point(left, top + height), Point(left + width, top + height), hud_color_, line_width);

    // draw the markers on either end
    line(hud_img, Point(left, top + height), Point(left, top), hud_color_, line_width);

    line(hud_img, Point(left + width, top + height), Point(left + width, top), hud_color_, line_width);

    float value_per_px = width / float(max_value - min_value);

    int extra_line_height;

    // draw little marker lines
    for (int i = min_value; i <= max_value; i += mark_increment) {

        int delta = max_value - min_value;
        int this_delta = delta - (i - min_value);
        int this_location = left + (float)this_delta * value_per_px;

        if (i == 0 || i == min_value || i == max_value) {
            extra_line_height = center_line_height_extra;
        } else {
            extra_line_height = 0;
        }
        line(hud_img, Point(this_location, top + height), Point(this_location, top - extra_line_height), hud_color_, line_width);
    }

    // draw the arrow

    // figure out where the arrow should be
    int sign_change;
    if (reverse_graph_direction) {
        sign_change = -1;
    } else {
        sign_change = 1;
    }

    float this_delta;
    if (zero_in_center) {
        this_delta = (max_value - min_value) - (sign_change*value - min_value);
    } else {
        this_delta = value - min_value;
    }

    int this_location = left + this_delta * value_per_px;

    // figure out if we're off-scale high
    if (this_location > left + width) {
        // off-scale high

        // draw an arrow like this >

        // top of the arrow
        line(hud_img, Point(left + width + arrow_height/3, top - arrow_gap - arrow_width/2), Point(left + width + arrow_height/3 - arrow_height, top - arrow_gap - arrow_width), hud_color_, line_width);

        // bottom of the arrow
        line(hud_img, Point(left + width + arrow_height/3, top - arrow_gap - arrow_width/2), Point(left + width + arrow_height/3 - arrow_height, top - arrow_gap), hud_color_, line_width);

        // vertical line of the arrow
        //line(hud_img, Point(left + width - arrow_height, top - arrow_gap - arrow_width), Point(left + width - arrow_height, top - arrow_gap), hud_color_, line_width);

    } else if (this_location < left) {
        // off-scale low

        // draw an arrow like this <

        // top of the arrow
        line(hud_img, Point(left - arrow_height/3, top - arrow_gap - arrow_width/2), Point(left - arrow_height/3 + arrow_height, top - arrow_gap - arrow_width), hud_color_, line_width);

        // bottom of the arrow
        line(hud_img, Point(left - arrow_height/3, top - arrow_gap - arrow_width/2), Point(left - arrow_height/3 + arrow_height, top - arrow_gap), hud_color_, line_width);

    } else {

        // right side of the arrow
        line(hud_img, Point(this_location + arrow_width/2, top - arrow_gap - arrow_height), Point(this_location, top - arrow_gap), hud_color_, line_width);

        // left side of the arrow
        line(hud_img, Point(this_location - arrow_width/2, top - arrow_gap - arrow_height), Point(this_location, top - arrow_gap), hud_color_, line_width);

        // top of the arrow
        line(hud_img, Point(this_location - arrow_width/2, top - arrow_gap - arrow_height), Point(this_location + arrow_width/2, top - arrow_gap - arrow_height), hud_color_, line_width);
    }

    // draw label
    int baseline = 0;
    Size text_size = getTextSize(label, text_font_, hud_font_scale_small_, text_thickness_, &baseline);

    PutHudTextSmall(hud_img, label, Point(left - text_gap - text_size.width, top + text_size.height/2 ));

    // draw value
    char accel_char[100];
    if (value < 0) {
        sprintf(accel_char, minus_format.c_str(), -value);
    } else {
        sprintf(accel_char, plus_format.c_str(), value);
    }
    string accel_str = accel_char;

    baseline = 0;
    text_size = getTextSize(accel_str, text_font_, hud_font_scale_small_, text_thickness_, &baseline);

    PutHudTextSmall(hud_img, accel_str, Point(left + width + text_gap, top + text_size.height/2));

}

void Hud::DrawAutonomous(Mat hud_img) {
    int top = 0.07 * hud_img.rows;
    int left;

    string str;

    if (is_autonomous_ == 1) {
        str = "AUTONOMOUS";
        left = 0.70 * hud_img.cols;
    } else {
        str = "MANUAL";
        left = 0.82 * hud_img.cols;
    }

    PutHudText(hud_img, str, Point(left, top));
}

void Hud::DrawCenterMark(Mat hud_img) {

    int radius = 0.020 * hud_img.rows;
    int line_size = 0.022 * hud_img.cols;
    int top_line_size = 0.011 * hud_img.cols;

    // draw a circle in the center
    circle(hud_img, Point(hud_img.cols/2, hud_img.rows/2), radius, hud_color_, box_line_width_);

    // draw the lines on both sides
    line(hud_img, Point(hud_img.cols/2 + radius + line_size, hud_img.rows/2), Point(hud_img.cols/2 + radius, hud_img.rows/2), hud_color_, box_line_width_);

    line(hud_img, Point(hud_img.cols/2 - radius - line_size, hud_img.rows/2), Point(hud_img.cols/2 - radius, hud_img.rows/2), hud_color_, box_line_width_);

    // draw the line on the top
    line(hud_img, Point(hud_img.cols/2, hud_img.rows/2 - top_line_size - radius), Point(hud_img.cols/2, hud_img.rows/2 - radius), hud_color_, box_line_width_);

}

void Hud::DrawThrottle(Mat hud_img) {

    int left = 70;
    int top = 50;

    DrawGraphIndicator(hud_img, left, top, "Thr", 0, 100, 25, "%.0f%%", "-%.0f%%", throttle_, false, false);
}

void Hud::DrawDateTime(Mat hud_img) {

    int left = 1 * hud_img.cols;
    int top = 1 * hud_img.rows;

    int text_gap = 1;

    char tmbuf[64], buf[64];

    // figure out what time the plane thinks it is
    struct tm *nowtm;
    time_t tv_sec = timestamp_ / 1000000.0;
    nowtm = localtime(&tv_sec);
    strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
    sprintf(buf, "%s", tmbuf);


    string datetime = buf;

    // put the date/time on the frame
    int baseline = 0;
    Size text_size = getTextSize(datetime, text_font_, hud_font_scale_small_, text_thickness_, &baseline);

    PutHudTextSmall(hud_img, datetime, Point(left - text_size.width - text_gap, top - text_size.height + baseline));

}
