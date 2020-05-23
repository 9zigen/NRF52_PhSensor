/***
** Created by Aleksey Volkov on 18.03.2020.
***/

#ifndef PHSENSOR_STORAGE_H
#define PHSENSOR_STORAGE_H

/* 4 bytes aligned */
typedef struct {
  uint32_t magic;
  uint32_t version;
  int16_t low_mv;
  int16_t mid_mv;
  int16_t hi_mv;
  uint16_t scan_interval;
} settings_t;

void storage_init();
void set_settings(int16_t low, int16_t mid, int16_t hi);

#endif //PHSENSOR_STORAGE_H
