#include <stdio.h>
#include "float.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/uart.h"
#include "hal/uart_types.h"
#include "c6_prototyper_core.h"
#include "app_settings.h"
#include "core_utils.h"
#include "windsonic_uart.h"
#include "windsonic_uart_config_report.h"

#define WINDSONIC_UART_BUF_SIZE 32  // for receiving messages over UART (NOT the data buffers)

/* Windsonic - polling over UART
Operating model is calls to take readings which are stored in a buffer followed by call to function which computes mean/sd (and resets the buffer index).
A circular buffer is used.

This is fixed to using UART2 = the LP UART (although used from HP core), which must use pins 5 (Tx) and 4 (Rx).

There is no support for changing the Windsonic operating mode; it must have previously been set up for UV format polling mode at the chosen baud rate.
The Windsonic configuration should look something like: M3,U1,O2,L1,P1,B3,H1,NQ,F1,E3,T3,S9,C2,G0,K50
Which gives polling responses like [02]Q,+000.01,-000.01,M,00,[03]36

*/

static const char* TAG = "WIND";

const char* request = "?Q";  // polling request - does not need CR/LF
uint8_t uart_buffer[WINDSONIC_UART_BUF_SIZE];  // characters from Windsonic
uint16_t buffer_ix = 0;  // index to store next value
uint16_t buffer_valid = 0;
float u_buffer[CONFIG_WINDSONIC_BUFF_LEN];  // cartesian U wind speed
float v_buffer[CONFIG_WINDSONIC_BUFF_LEN];  // V wind speed

/* NO Settings - the Windsonic head is assumed to be pre-calibrated */

esp_err_t wind_init(){
    #ifdef CONFIG_WINDSONIC_LOG_LEVEL
    esp_log_level_set(TAG, CONFIG_WINDSONIC_LOG_LEVEL);
    #else
    esp_log_level_set(TAG, ESP_LOG_WARN);
    #endif

    ESP_LOGD(TAG, "Initialising Windsonic UART");
    uart_config_t uart_config = {
        .baud_rate = CONFIG_WINDSONIC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = LP_UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(LP_UART_NUM_0, WINDSONIC_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err == ESP_OK){
        err = uart_param_config(LP_UART_NUM_0, &uart_config);
        if (err == ESP_OK){
            err = uart_set_pin(LP_UART_NUM_0, 5, 4, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);  // needed even though the pins are not changeable for LP UART
        }
    }
    if (err != ESP_OK){
        ESP_LOGE(TAG, "Uart init %s", esp_err_to_name(err));
    }

    return err;
}

// takes a reading by polling the windsonic, storing the U,V values into the buffers
esp_err_t wind_take_reading(){
    esp_err_t err = ESP_OK;
    float u, v;
    int status;

    uart_flush_input(LP_UART_NUM_0);  // flush input in case dangling from the past
    int len_tx = uart_write_bytes(LP_UART_NUM_0, request, 2);
    if (len_tx == 2){
        // Read data from the UART - typical interval for completion of poll is 120-130ms
        int len_rx = uart_read_bytes(LP_UART_NUM_0, uart_buffer, 27, 1500 / portTICK_PERIOD_MS);
        ESP_LOGD(TAG, "Received (STX & ETX hidden): %s", (char*)(uart_buffer+1));  // start after 0x02 = STX. The ETX is lost from stdout so this shows the checksum chars
        if (len_rx == 27) {
            uart_buffer[len_rx] = '\0';
            // compute checksum - exclusive OR of bytes between STX and ETX. Source checksum is 2 chars after ETX, being the ascii encoding of a hex value !!
            uint8_t chk_source;
            sscanf((char*)(uart_buffer + 25), "%hhX", &chk_source);
            uint8_t chk = 0;
            for (uint8_t i=1; i<24; i++){
                chk ^= uart_buffer[i];
            }
            if (chk == chk_source) {
                // decode parts, indexing in from the start; the commas cause atof to break off parsing
                u = atof((char*)(uart_buffer + 3));
                v = atof((char*)(uart_buffer + 11));
                status = atoi((char*)(uart_buffer + 21));
                ESP_LOGD(TAG, "U: %.2f, V: %.2f, S: %i", u, v, status);
                if (status != 0) {
                    ESP_LOGW(TAG, "Bad status from Windsonic %u; discarding readings", status);
                    err = ESP_FAIL;
                }
            } else {
                ESP_LOGW(TAG, "Checksum fail: %02X vs %02X received", chk, chk_source);
                err = ESP_ERR_INVALID_CRC;
            }
        } else {
            ESP_LOGW(TAG, "Bad RX - expected length 27 (incl STX & ETX) but got %u", len_rx);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGW(TAG, "TX failed retval=%i", len_tx);
        err = ESP_FAIL;
    }

    if (err == ESP_OK){
        u_buffer[buffer_ix] = u;
        v_buffer[buffer_ix] = v;
        buffer_ix = (buffer_ix + 1) % CONFIG_WINDSONIC_BUFF_LEN;
        if (buffer_valid < CONFIG_WINDSONIC_BUFF_LEN) ++buffer_valid;    
    }
    
    return err;
}

// Read un-used and take mean/sd.
// min_samples sets minimum for a valid measurement. max_samples would normally be the expected number of readings taken but may be smaller if bursts just before the logging event are wanted
// In the event that there are not enough un-read entries in the buffer, or if any are invalid the returned mean value will be set to the "NA" placeholder: FLOAT_NA
esp_err_t wind_process_samples(float * speed, float * bearing, float *sd, u_int16_t min_samples, u_int16_t max_samples){
    
    esp_err_t err = ESP_OK;

    // failure case fallbacks only over-written if all OK
    *speed = FLOAT_NA;
    *bearing = FLOAT_NA;
    *sd = FLOAT_NA;

    if (buffer_valid >= min_samples){
        ESP_LOGD(TAG, "Computing speed/bearing/sd (buffer_ix = %lu, buffer_valid = %lu)", buffer_ix, buffer_valid);
        // compute the mean cartesian vector from the series of U,V speeds then render this to polar form
        uint16_t use_n = MIN(max_samples, buffer_valid);
        float u = 0;
        float v = 0;
        uint16_t ix;
        for (uint16_t i = 1; i <= use_n ; i++){
            // funky casting of index is needed so modulo works as expected. ix is 16 bit to allow for long buffer
            ix = ((uint16_t)(buffer_ix - i)) % CONFIG_WINDSONIC_BUFF_LEN;
            u += u_buffer[ix];
            v += v_buffer[ix];
        }
        u /= use_n;  // scale to mean
        v /= use_n;
        *speed = sqrt(u*u + v*v);
        // bearing positive U is towards the North, positive V is towards the West. Towards S is bearing =0, toward W = 90, toward N =180, toward E =270
        if (v > 0) {  // bearings from 0 to 180
            *bearing = acos(-u / *speed) * 57.30;  // factor converts from radian to degree
        } else {
            *bearing = 180 + acos(u / *speed) * 57.30;
        }
        // SD only if at least 3 samples
        if (use_n >= 3) {
            float u_sq = 0;
            float v_sq = 0;
            for (uint16_t i = 1; i <= use_n ; i++){
                ix = ((uint16_t)(buffer_ix - i)) % CONFIG_WINDSONIC_BUFF_LEN;
                u_sq += pow(u - u_buffer[ix], 2);
                v_sq += pow(v - v_buffer[ix], 2);
            }
            *sd = sqrt(u_sq / use_n + v_sq / use_n);  // root of sum of cartesian variances
        }

        ESP_LOGD(TAG, "Mean Speed: %.3fm/s @ bearing %.1f, SD: %.3fm/s", *speed, *bearing, *sd);


    } else {
        ESP_LOGW(TAG, "Insufficient samples to compute mean speed and bearing. Had %u, needed %u", buffer_valid, min_samples);
        err = ESP_ERR_INVALID_SIZE;
    }

    buffer_valid = 0;  // set as "all used" in the buffer, irrespective of success/failure

    return err;
}