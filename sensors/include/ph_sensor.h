/***
** Created by Aleksey Volkov on 2019-02-16.
***/

#ifndef PHSENSOR_PH_SENSOR_H
#define PHSENSOR_PH_SENSOR_H

void start_ph_sensor();
double get_ph_value();

typedef enum {
  PH_LOW_CALIBRATION,
  PH_MID_CALIBRATION,
  PH_HI_CALIBRATION
} ph_calibration_t;

void do_ph_calibration(ph_calibration_t type);

void power_ph_sensor(bool on);

#endif //PHSENSOR_PH_SENSOR_H
