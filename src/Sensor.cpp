#include "Sensor.h"
#include <Arduino.h>

void sensorBegin(unsigned char pin1, unsigned char pin2) {
    pinMode(pin1, INPUT_PULLUP); pinMode(pin2, INPUT_PULLUP);
    attachInterrupt(pin1, tickInterrupCounter1, RISING);
    attachInterrupt(pin2, tickInterrupCounter2, RISING);
}



void tickInterrupCounter1() {
    sensorLeft.ticks++;
}


void tickInterrupCounter2() {
    sensorRight.ticks++;
}
