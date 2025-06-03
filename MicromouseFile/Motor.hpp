#pragma once
#include <Arduino.h>
#include "math.h"

namespace mtrn3100 {
// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pina, uint8_t in1, uint8_t pwm_pinb, uint8_t in2 ) :  pwm_pin1(pwm_pina), dir_pin1(in1), pwm_pin2(pwm_pinb), dir_pin2(in2) {
        pinMode(pwm_pin1, OUTPUT);
        pinMode(dir_pin1, OUTPUT);
        pinMode(pwm_pin2, OUTPUT);
        pinMode(dir_pin2, OUTPUT);
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWMRight(int16_t pwm) {

      // TODO: Output digital direction pin based on if input signal is positive or negative.
      // TODO: Output PWM signal between 0 - 255
      if (pwm > 255) {
      pwm = 255;
      }

      if (pwm <= 0) {
        // This ensures that there is still a signal to make it move (but makes sure it is readable) 
        pwm = -pwm;
        // This switches the direction of rotation of the motor
        // when using 2 motors, directions need to be reversed to go in the same direction
        // e.g. LOW LOW for pin1 and 2 will get it to turn
        digitalWrite(dir_pin1, LOW);
        
        // Serial.println("this is activated");
        // Serial.println(pwm);
        // This one makes the motor spin at that pwm speed
        analogWrite(pwm_pin1, pwm);
      }
      else if (pwm <= 255) {

        digitalWrite(dir_pin1, HIGH);
        analogWrite(pwm_pin1, pwm);
      }
    }

    void setPWMLeft(int16_t pwm) {
       if (pwm > 255) {
      pwm = 255;
      }

      if (pwm <= 0) {
        // This ensures that there is still a signal to make it move (but makes sure it is readable) 
        pwm = -pwm;
        digitalWrite(dir_pin2, HIGH);
        analogWrite(pwm_pin2, pwm);
      }

      else if (pwm <= 255) {

        digitalWrite(dir_pin2, LOW);
        analogWrite(pwm_pin2, pwm);
      }
    }
    
    void turnRight(int16_t pwm){
      setPWMRight(-pwm);
      setPWMLeft(pwm);
    }

    void turnLeft(int16_t pwm){
      setPWMRight(pwm);
      setPWMLeft(-pwm);
    }

    void moveForward(int16_t pwm ){
      setPWMLeft(pwm);
      setPWMRight(pwm);
    }

private:
    const uint8_t pwm_pin1;
    const uint8_t dir_pin1;
    const uint8_t pwm_pin2;
    const uint8_t dir_pin2;
};

};  // namespace mtrn3100