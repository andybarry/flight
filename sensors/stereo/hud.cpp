#include "hud.hpp"

Hud::Hud() {
    airspeed = 0;
}


/**
 * Draws the HUD (Heads Up Display)
 * 
 * @param _input_image image to draw the HUD on
 * 
 * @retval image with the HUD drawn
 */
void DrawHud(InputArray _input_image, OutputArray _output_image) {
    Mat input_image = _input_image.getMat();
    
    _output_image.create(input_image.size(), input_image.type());
    Mat output_image = _output_image.getMat();
    
    input_image.copyTo(output_image);
}

