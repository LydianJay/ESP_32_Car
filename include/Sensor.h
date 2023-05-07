#ifndef _SENSOR_H
#define _SENSOR_H

struct SpeedSensorData {
  unsigned short ticks;
  unsigned short maxTicks;
  unsigned char pin;
};

extern SpeedSensorData sensorLeft, sensorRight;

void sensorBegin(unsigned char pin1, unsigned char pin2);

void tickInterrupCounter1();

void tickInterrupCounter2();


#endif