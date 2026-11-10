#pragma once

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

/* ---------------------------- System config ---------------------------- */
#define GPS_TX GPIO_NUM_14 // ESP RX  <- GPS TX
#define GPS_RX GPIO_NUM_12 // ESP TX  -> GPS RX

#define GPS_UART_NUM       UART_NUM_1
#define GPS_BAUDRATE       115200
#define GPS_RX_CHUNK       128
#define GPS_BUFF_SIZE      4096
#define GPS_SAMPLE_RATE_MS 200 // 5 Hz

#define RTK_MAX_SV 16
/* ---------------------------- UBX config ---------------------------- */
#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_MAX_PAYLOAD 1024
#define UBX_HDR_LEN     4 // class, id, len_l, len_h
#define UBX_CK_LEN      2

#define UBX_ACK_TIMEOUT_MS 1500
#define UBX_ACK_RETRIES    3

#define UBX_POSLLH BIT(0)
#define UBX_VELNED BIT(1)
#define UBX_STATUS BIT(2)
#define HAVE_ALL   (UBX_STATUS | UBX_VELNED | UBX_POSLLH)

#define CAL_MIN_SAMPLES 20
#define CAL_MAX_SAMPLES 200
#define CAL_VACC_MAX_MM 5000

#define GPS_BIT BIT(0)
#define RTK_BIT BIT(1)

/* ---------------------------- GPS config ---------------------------- */
#define GPS_DATA_ACTIVE 0
#define RTK_DATA_ACTIVE 1

/* ---------------------------- Handles ---------------------------- */
extern portMUX_TYPE xGPSMutex;
extern TaskHandle_t xTaskLog;

/* ---------------------------- Sample structures ---------------------------- */
typedef struct {
    int32_t latitude;     // deg * 1e-7, + = N
    int32_t longitude;    // deg * 1e-7, + = E
    float   altitude;     // m above calibrated pad altitude
    float   vel_vertical; // m/s, up positive
    uint8_t sAcc;         // speed accuracy * 10 (m/s)
    uint8_t fix;          // gpsFix type, 0 if not OK
} gps_sample_t;

typedef struct {
    uint32_t iTOW;                         // ms
    int16_t  week;                         // weeks
    uint8_t  numSV;                        // # of satellites following
    double   carrier_phase[RTK_MAX_SV];    // L1 cycles
    double   pseudorange_meas[RTK_MAX_SV]; // m
    float    doppler_meas[RTK_MAX_SV];     // Hz
    uint8_t  sv_id[RTK_MAX_SV];            // Space Vehicle Number
    uint8_t  meas_quality[RTK_MAX_SV];     // Nav Measurements Quality Indicator
    int8_t   signal_strength[RTK_MAX_SV];  // dbHz
    uint8_t  lli[RTK_MAX_SV];              // Loss of lock indicator (RINEX)
} rtk_sample_t;

typedef enum {
    ACK_NONE,
    ACK_OK,
    ACK_NAK,
} ubx_ack_t;

typedef struct {
    uint8_t class;
    uint8_t id;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t len;
} ubx_cmd_t;

typedef struct {
    uint8_t class;
    uint8_t   id;
    ubx_ack_t response;
} ack_state_t;

typedef struct {
    uint8_t  numSV;
    uint16_t pDOP; // x0.01
    bool     valid;
    char     swVersion[30];
    char     hwVersion[10];
} gps_info_t;

typedef struct {
    uint32_t itow;
    uint8_t  have;
    bool     fix_ok;
} epoch_t;

typedef struct {
    float    hmsl_m;
    uint32_t vacc_mm;
    float    mean;
    uint32_t cnt;
} alt_cal_t;

extern gps_sample_t    gps_sample_g;
extern rtk_sample_t    rtk_sample_g;
extern ack_state_t     ack_state;
extern gps_info_t      gps_info;
extern epoch_t         epoch;
extern alt_cal_t       alt_cal;
extern const ubx_cmd_t gps_config[];
extern const size_t    gps_config_len;