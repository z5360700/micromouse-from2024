#pragma once
#include <VL6180X.h>

namespace mtrn3100 {

class ThreeLidar {
public:
    ThreeLidar(int pin1, int pin2, int pin3)
        : sensor1_pin(pin1), sensor2_pin(pin2), sensor3_pin(pin3) {
        pinMode(sensor1_pin, OUTPUT);
        pinMode(sensor2_pin, OUTPUT);
        pinMode(sensor3_pin, OUTPUT);
        digitalWrite(sensor1_pin, LOW);
        digitalWrite(sensor2_pin, LOW);
        digitalWrite(sensor3_pin, LOW);
    }

    void init() {        
        Wire.begin();

        // Initialize first sensor and change address
        digitalWrite(sensor1_pin, HIGH);
        delay(50);
        sensor1.init();
        sensor1.configureDefault();
        sensor1.setTimeout(250);
        sensor1.setAddress(0x58);
        delay(50);

        // Initialize second sensor and change address
        digitalWrite(sensor2_pin, HIGH);
        delay(50);
        sensor2.init();
        sensor2.configureDefault();
        sensor2.setTimeout(250);
        sensor2.setAddress(0x2A);
        delay(50);

        // Initialize third sensor and change address
        digitalWrite(sensor3_pin, HIGH);
        delay(50);
        sensor3.init();
        sensor3.configureDefault();
        sensor3.setTimeout(250);
        sensor3.setAddress(0x78);
        delay(50);
    }

    int readLeftSensor() {
        return sensor1.readRangeSingleMillimeters();
    }

    int readForwardSensor() {
        return sensor2.readRangeSingleMillimeters();
    }

    int readRightSensor() {
        return sensor3.readRangeSingleMillimeters();
    }

    bool timeoutOccurredSensor1() {
        return sensor1.timeoutOccurred();
    }

    bool timeoutOccurredSensor2() {
        return sensor2.timeoutOccurred();
    }

    bool timeoutOccurredSensor3() {
        return sensor3.timeoutOccurred();
    }

private:
    VL6180X sensor1;
    VL6180X sensor2;
    VL6180X sensor3;
    int sensor1_pin;
    int sensor2_pin;
    int sensor3_pin;
};

}  // namespace mtrn3100
