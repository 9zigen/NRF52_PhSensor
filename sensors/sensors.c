/***
** Created by Aleksey Volkov on 16/11/2018.
***/

#include <libraries/timer/app_timer.h>
#include <ble_bas.h>
#include <ble_services.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_clock.h"
#include "nrfx_ppi.h"
#include "nrf_gpio.h"

#include "custom_board.h"
#include "bat_sensor.h"
#include "ph_sensor.h"
#include "sensors.h"
#include "ntc_temperature_sensor.h"
#include "timers.h"

/* current sensor */
uint8_t sensor_index             = 0;


bool battery_sensor_ready        = false;
bool temperature_sensor_ready    = false;
bool ph_sensor_ready             = false;

bool battery_sensor_notify       = false;
bool temperature_sensor_notify   = false;
bool ph_sensor_notify            = false;

/* Slow read sensors timer handler */
void read_sensors_timer_handler(void *p_context)
{
  if(p_context)
  {
    /* fast run */
    if (all_sensors_ready())
    {
      /* restart read after delay 30 min */
      read_sensor_timer_stop();
      read_sensor_timer_start(false);

      NRF_LOG_INFO("update advertising data");
      /* update advertising data */
      advertising_update();
      reset_all_sensors();
      return;
    }

    NRF_LOG_INFO("-------------- READ SENSOR --------------");
    read_sensors();
  } else {
    NRF_LOG_INFO("-------------- DELAY ELAPSED ------------");
    /* delay elapsed */
    read_sensor_timer_stop();
    read_sensor_timer_start(true);
  }

}

void update_advertising_timer_handler(void *p_context)
{
  UNUSED_PARAMETER(p_context);
  if (all_sensors_ready())
  {
    advertising_update();
  }
}

/* update characteristics timer handler */
void update_characteristics_timer_handler(void *p_context) {
  UNUSED_PARAMETER(p_context);

  NRF_LOG_INFO("-------------- UPDATE CH ------------");
  /* read only one sensor, and SET Sensor Ready */
  read_sensors();

  /* update only one sensor, readied in previous call */
  if (battery_sensor_notify && sensor_ready(BATTERY_SENSOR))
  {
    update_sensors_service(BATTERY_SENSOR);
    reset_sensor(BATTERY_SENSOR);
  }
  else if (temperature_sensor_notify && sensor_ready(TEMPERATURE_SENSOR))
  {
    update_sensors_service(TEMPERATURE_SENSOR);
    reset_sensor(TEMPERATURE_SENSOR);
  }
  else if (ph_sensor_notify && sensor_ready(PH_SENSOR))
  {
    update_sensors_service(PH_SENSOR);
    reset_sensor(PH_SENSOR);
  }

}

/* Sensors Order
 * 0 - Battery SAADC
 * 1 - NTC SAADC
 * 2 - PH SAADC
 * */
void read_sensors() {


  NRF_LOG_INFO("### INDEX %d", sensor_index);

  if (nrfx_saadc_is_busy()) {
    NRF_LOG_INFO("SAADC is Busy");
    return;
  }

  if (sensor_index == 0) {
    read_battery_voltage();
    sensor_index++;

    /* power on: OP amplifier, voltage divider, ph probe */
    power_ph_sensor(true);

  } else if (sensor_index == 1) {
    read_ntc_temperature();
    sensor_index++;

  } else if (sensor_index == 2) {
    start_ph_sensor();
    sensor_index = 0;

  }
}

void set_sensor_ready(sensor_ready_t sensor)
{
  switch (sensor)
  {
    case BATTERY_SENSOR_READY:
      battery_sensor_ready = true;
      break;

    case PH_SENSOR_READY:
      ph_sensor_ready = true;
      break;

    case TEMPERATURE_SENSOR_READY:
      temperature_sensor_ready = true;
      break;

    default:
      break;
  }
}

/* Check if sensor data ready to read */
bool sensor_ready(sensor_t sensor_id)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR: return battery_sensor_ready;
    case TEMPERATURE_SENSOR: return temperature_sensor_ready;
    case PH_SENSOR: return ph_sensor_ready;
    default: return false;
  }
}

bool all_sensors_ready()
{
  if (battery_sensor_ready && temperature_sensor_ready && ph_sensor_ready)
    return true;
  else
    return false;
}

void reset_sensor(sensor_t sensor_id)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR:
      battery_sensor_ready = false;
      break;
    case TEMPERATURE_SENSOR:
      temperature_sensor_ready = false;
      break;
    case PH_SENSOR:
      ph_sensor_ready = false;
      break;
    default:
      break;
  }
}

void reset_all_sensors()
{
  battery_sensor_ready = false;
  temperature_sensor_ready = false;
  ph_sensor_ready = false;
}

bool is_any_sensor_notify()
{
  if (battery_sensor_notify || temperature_sensor_notify || ph_sensor_notify)
    return true;
  else
    return false;
}

void set_sensor_notify(sensor_t sensor_id, bool notify)
{
  switch (sensor_id)
  {
    case BATTERY_SENSOR:
      battery_sensor_notify = notify;
      break;
    case TEMPERATURE_SENSOR:
      temperature_sensor_notify = notify;
      break;
    case PH_SENSOR:
      ph_sensor_notify = notify;
      break;
    default:
      break;
  }
}