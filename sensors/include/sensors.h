/***
** Created by Aleksey Volkov on 16/11/2018.
***/

#ifndef SOILSENSOR_SENSORS_H
#define SOILSENSOR_SENSORS_H

#include "nrfx.h"

typedef enum {
  BATTERY_SENSOR,
  TEMPERATURE_SENSOR,
  PH_SENSOR,

} sensor_t;

typedef enum {
  BATTERY_SENSOR_READY,
  TEMPERATURE_SENSOR_READY,
  PH_SENSOR_READY,

} sensor_ready_t;


/* Add status %Sensor% READY for main task flow */
void set_sensor_ready(sensor_ready_t sensor);

/* Get status %Sensor% READY in main task flow */
sensor_ready_t get_sensor_ready();

/* check if sensor Ready */
bool sensor_ready(sensor_t sensor_id);

/* Reset sensor */
void reset_sensor(sensor_t sensor_id);

/* ALL sensors Ready */
bool all_sensors_ready();

/* Reset ALL sensors status */
void reset_sensors();

/* timer handlers */
void update_advertising_timer_handler(void *p_context);
void read_sensors_timer_handler(void *p_context);
void update_characteristics_timer_handler(void *p_context);

/* Separate sead sensors, one call -> one sensor */
void read_sensors();

bool is_any_sensor_notify();
void set_sensor_notify(sensor_t sensor_id, bool notify);

void reset_all_sensors();

#endif //SOILSENSOR_SENSORS_H
