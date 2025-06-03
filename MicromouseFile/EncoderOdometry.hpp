// COPIED FROM LABS

#pragma once
#include <Arduino.h>
namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), L(wheelBase), lastLPos(0), lastRPos(0) {}

    //TODO: COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {

        float delta_left_radians = -(lastLPos - leftValue); // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = -(lastRPos - rightValue); // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 

        //TODO: Calculate the foward kinematics
        x += (((R*delta_left_radians)/2)+((R*delta_right_radians)/2))*cos(h);
        y += (((R*delta_left_radians)/2)+((R*delta_right_radians)/2))*sin(h);
        h += -(R*delta_left_radians)/(L) + (R*delta_right_radians)/(L);
        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, L;
    float lastLPos, lastRPos;
};

}
