/***
** Created by Aleksey Volkov on 23/11/2018.
***/

#ifndef SOILSENSOR_CUSTOM_BOARD_H
#define SOILSENSOR_CUSTOM_BOARD_H

#include "nrf_gpio.h"

/* RGB LED */
#define               LED_R         NRF_GPIO_PIN_MAP(0,24)
#define               LED_G         NRF_GPIO_PIN_MAP(0,23)
#define               LED_B         NRF_GPIO_PIN_MAP(0,22)

/* White LED */
#define               LED_W         NRF_GPIO_PIN_MAP(0,25)

/* Button */
#define               BTN_PIN       NRF_GPIO_PIN_MAP(0,31)

/* NTC Thermistor 10K */
#define               NTC_POWER_PIN NRF_GPIO_PIN_MAP(0,2)
#define               NTC_ADC_PIN   NRF_GPIO_PIN_MAP(0,3)

/* PH Sensor */
#define               PH_ON_PIN     NRF_GPIO_PIN_MAP(0,6)
#define               PH_ADC_PIN    NRF_GPIO_PIN_MAP(0,4)

/* Serial Interface */
#define               TX_PIN        NRF_GPIO_PIN_MAP(0,14)
#define               RX_PIN        NRF_GPIO_PIN_MAP(0,13)

#endif //SOILSENSOR_CUSTOM_BOARD_H
