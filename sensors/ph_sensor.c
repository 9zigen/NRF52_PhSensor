/***
** Created by Aleksey Volkov on 2019-02-16.
***/

#include <pwm.h>
#include <timers.h>
#include <storage.h>
#include "bsp/custom_board.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_saadc.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "nrf_gpio.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"

#include <sensors/include/ntc_temperature_sensor.h>
#include <sensors/include/bat_sensor.h>
#include <sensors/include/sensors.h>
#include "ph_sensor.h"

#define SAMPLES_IN_BUFFER     2
#define ADC_COMPENSATION      4
#define ADC_RES_12BIT         4095
#define ADC_REF_VOLTAGE_MV    600
#define KELVIN                273.15
#define MOSFET_DROP_MV        0          /* short bnc read 0mv */

/* Macro to convert the result of ADC conversion in millivolts. */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_MV) / ADC_RES_12BIT) * ADC_COMPENSATION)

/* ADC */
static nrf_saadc_value_t            m_buffer_pool[2][SAMPLES_IN_BUFFER];
int16_t  ph_milli_volts           = 0;
int16_t calibration_neutral_mv    = 0;
int16_t calibration_low_mv        = 0;
int16_t calibration_high_mv       = 0;

bool need_low_calibration         = false;
bool need_mid_calibration         = false;
bool need_hi_calibration          = false;

/* PPI */
static nrf_ppi_channel_t            m_ppi_channel;

/* TIMER */
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1);


static void timer_handler(nrf_timer_event_t event_type, void * p_context) {}

static void saadc_sampling_event_init(void)
{
  ret_code_t err_code;

  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  timer_cfg.frequency = NRF_TIMER_FREQ_500kHz;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
  APP_ERROR_CHECK(err_code);

  /* setup m_timer for compare event to 200us, start SAADC */
  uint32_t ticks_saadc = nrfx_timer_us_to_ticks(&m_timer, 5000);
  nrfx_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks_saadc, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

  /* Start Timer */
  nrfx_timer_enable(&m_timer);

  /* get addresses for event and task short */
  uint32_t timer_compare_event_ch0_addr = nrfx_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
  uint32_t saadc_sample_task_addr       = nrfx_saadc_sample_task_get();

  /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
  err_code = nrfx_ppi_channel_alloc(&m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_assign(m_ppi_channel, timer_compare_event_ch0_addr, saadc_sample_task_addr);
  APP_ERROR_CHECK(err_code);
}

//static void saadc_sampling_event_enable(void)
//{
//  ret_code_t err_code = nrfx_ppi_channel_enable(m_ppi_channel);
//  APP_ERROR_CHECK(err_code);
//}

static void ph_sensor_deinit(void)
{
  ret_code_t err_code;
  err_code = nrfx_ppi_channel_disable(m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  err_code = nrfx_ppi_channel_free(m_ppi_channel);
  APP_ERROR_CHECK(err_code);

  nrfx_timer_disable(&m_timer);
  nrfx_timer_uninit(&m_timer);

  nrfx_saadc_uninit();
  NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);  //Disable the SAADC interrupt
  NVIC_ClearPendingIRQ(SAADC_IRQn);

  /* power off OP AMP */
  power_ph_sensor(false);
}

/* SAADC Calibration/Scan complete */
static void ph_saadc_callback(nrfx_saadc_evt_t const * p_event)
{
  if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
  {
    ret_code_t err_code;
    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    /* Enable PPI channel */
    err_code = nrfx_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("PH SAADC calibration complete!");

  } else if (p_event->type == NRFX_SAADC_EVT_DONE)
  {
    ret_code_t err_code;

    err_code = nrfx_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    for (uint8_t i = 0; i < p_event->data.done.size; i++)
    {
      NRF_LOG_INFO("PH SAADC RAW   %d", p_event->data.done.p_buffer[i]);
    }

    int16_t mid_milli_volts = (int16_t)((get_battery_milli_volts() / 2) - MOSFET_DROP_MV);
    ph_milli_volts = (uint16_t)ADC_RESULT_IN_MILLI_VOLTS(p_event->data.done.p_buffer[1]) - mid_milli_volts;
    NRF_LOG_INFO("PH SAADC MV %d", ph_milli_volts);
    NRF_LOG_INFO("PH SAADC PH " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(get_ph_value()));

    set_sensor_ready(PH_SENSOR_READY);
    ph_sensor_deinit();

    if (need_low_calibration)
    {
      calibration_low_mv = ph_milli_volts;
      need_low_calibration = false;
      NRF_LOG_INFO("PH LOW Calibration new mv %d", calibration_low_mv);
      /* save to nvram */
      set_settings(calibration_low_mv, calibration_neutral_mv, calibration_high_mv);
      led_indication_set(LED_INDICATE_SUCCESS);
    }

    else if (need_mid_calibration)
    {
      calibration_neutral_mv = ph_milli_volts;
      need_mid_calibration = false;
      NRF_LOG_INFO("PH Mid Calibration mv %d", calibration_neutral_mv);
      /* save to nvram */
      set_settings(calibration_low_mv, calibration_neutral_mv, calibration_high_mv);
      led_indication_set(LED_INDICATE_SUCCESS);
    }

    else if (need_hi_calibration)
    {
      calibration_high_mv = ph_milli_volts;
      need_hi_calibration = false;
      NRF_LOG_INFO("PH High Calibration mv %d", calibration_high_mv);
      /* save to nvram */
      set_settings(calibration_low_mv, calibration_neutral_mv, calibration_high_mv);
      led_indication_set(LED_INDICATE_SUCCESS);
    }

  }
}

/* SAADC setup for PH calculation */
static void ph_saadc_init(void)
{
  nrfx_saadc_uninit();

  ret_code_t err_code;

  /* Configure SAADC */
  nrfx_saadc_config_t saadc_config;
  saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT;        //Set SAADC resolution to 12-bit. This will make the SAADC output values from 0 (when input voltage is 0V) to 2^12=4096 (when input voltage is 3.6V for channel gain setting of 1/6).
  saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;
  saadc_config.oversample = NRF_SAADC_OVERSAMPLE_128X;

  err_code = nrfx_saadc_init(&saadc_config, ph_saadc_callback);
  APP_ERROR_CHECK(err_code);

  /* PH */
  nrf_saadc_channel_config_t channel_config =
      NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
  channel_config.gain = NRF_SAADC_GAIN1_4;
  channel_config.burst = NRF_SAADC_BURST_ENABLED;

  err_code = nrfx_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  /* Start calibration */
  nrfx_saadc_calibrate_offset();

  /* Start conversion */
//  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
//  APP_ERROR_CHECK(err_code);
//
//  err_code = nrfx_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
//  APP_ERROR_CHECK(err_code);
}

/* power on ph sensor */
void power_ph_sensor(bool on)
{
  if (on)
  {
    nrf_gpio_cfg_output(PH_ON_PIN);
    /* set pin LOW to power OP Amplifier thought p-mosfet */
    nrf_gpio_pin_clear(PH_ON_PIN);
  } else {
    nrf_gpio_cfg_input(PH_ON_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_input_disconnect(PH_ON_PIN);
  }

}

void start_ph_sensor()
{
  ph_saadc_init();
  saadc_sampling_event_init();
//  saadc_sampling_event_enable();
}

uint16_t get_ph_raw_value ()
{
  return ph_milli_volts;
}

/* PH sensor
 * formula:
 * value = 7 - (ph7_zero_mv  - sensor_mv) * slope
 * Slope (10°C) = slope(24°C) * (10 + 273.15) / (24 + 273.15)
 * 1. Choose two buffers, let say pH1 = 7.00 and pH2 = 4.00 and measure the corresponding E1 and E2 (in mV).
 * 2. For a sample with pHx, you measure Ex.
 * 3. Calculate pHx = pH1 + (Ex – E1)(pH2 – pH1)/(E2-E1)
 * */
double get_ph_value()
{
  double current_temperature = get_ntc_temperature();

  /* ideal ph probe slope */
  double slope = 59.18; /* mv/ph */

  /* real ph probe slope */
  if (ph_milli_volts > -25)  /* PH < 7.0 */
  {
    if (calibration_neutral_mv && calibration_low_mv) {
      slope = (calibration_low_mv - calibration_neutral_mv) / (6.86 - 4.01);

    }
  } else {                 /* PH > 7.0 */
    if (calibration_neutral_mv && calibration_high_mv) {
      slope = (calibration_high_mv - calibration_neutral_mv) / (9.18 - 6.86);
    }
  }

  /* temperature corrected slope
   * 50C - 64mV/pH Unit
   * 25C - 59mV/pH Unit
   * 0C  - 54mV/pH Unit */
  double slope_offset =  (25 - current_temperature) * -0.1985;
  double corrected_slope = slope + slope_offset;
  NRF_LOG_INFO("Slope " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(corrected_slope));
//  double corrected_slope = slope * ((current_temperature + KELVIN) / (25 + KELVIN));

  /* */
  double value = 7.0 - ((double)(ph_milli_volts) / corrected_slope);
  return value;
}

void do_ph_calibration(ph_calibration_t type)
{
  /* check if notification disabled */
//  if (!is_any_sensor_notify())
//  {
//    /* restart the timer in fast mode to quickly read all the sensors
//     * battery voltage and temperature need for proper ph calculation */
//    read_sensor_timer_stop();
//    read_sensor_timer_start(true);
//  }

  /* read all sensors to proper PH calculation */
  read_sensor_timer_stop();
  read_sensor_timer_start(true);

  need_low_calibration = false;
  need_mid_calibration = false;
  need_hi_calibration  = false;

  switch (type)
  {
    case PH_LOW_CALIBRATION:
      need_low_calibration = true;
      led_indication_set(LED_INDICATE_FAST_RED);
      break;
    case PH_MID_CALIBRATION:
      need_mid_calibration = true;
      led_indication_set(LED_INDICATE_FAST_GREEN);
      break;
    case PH_HI_CALIBRATION:
      need_hi_calibration  = true;
      led_indication_set(LED_INDICATE_FAST_BLUE);
      break;
    default:
      break;
  }

  /* temporary disable led */
  //led_indication_set(LED_INDICATE_NONE);
}

uint8_t get_ph_calibration_status()
{
  return need_low_calibration || need_mid_calibration || need_hi_calibration;
}

void set_ph_calibration(int16_t low, int16_t mid, int16_t hi)
{
  calibration_low_mv = low;
  calibration_neutral_mv = mid;
  calibration_high_mv = hi;
}