
#include "esp_err.h"

#ifndef WINDSONIC_UART_H
#define WINDSONIC_UART_H

esp_err_t wind_init();
esp_err_t wind_take_reading();
esp_err_t wind_process_samples(float * speed, float * bearing, float *sd, u_int16_t min_samples, u_int16_t max_samples);
#endif