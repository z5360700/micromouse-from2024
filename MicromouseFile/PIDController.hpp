#pragma once
#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float tolerance) 
        : kp(kp), ki(ki), kd(kd), tolerance(tolerance), integral(0), prev_error(0), prev_time(0), errorCount(0) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;  // Convert to seconds
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // Calculate PID output
        integral += error * dt;
        derivative = (error - prev_error) / dt;
        output = (kp * error) + (ki * integral) + (kd * derivative);

        // Check if error is within tolerance
        if (abs(error) < tolerance) {
            errorCount++;
        } else {
            errorCount = 0;
        }

        prev_error = error;

        //Serial.println(output);
        return constrain(output, -255, 255);
    }

    // Function to update internal PID parameters
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    // Function used to return the last calculated error.
    float getError() {
        return error;
    }

    // This must be called before trying to achieve a setpoint.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;
        integral = 0;  // Reset integral term when setting a new target
        prev_error = 0; // Reset previous error when setting a new target
        errorCount = 0; // Reset error count when setting a new target
    }

    // Checks if the controller has settled
    bool isSettled() {

        return (abs(error) < tolerance) || (errorCount > 100);
    }

private:
    uint32_t prev_time, curr_time;
    float dt;

    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error;
    float setpoint;
    float zero_ref;
    float tolerance;

    uint32_t errorCount; // Tracks the number of consecutive iterations with the error within tolerance
};

}  // namespace mtrn3100

