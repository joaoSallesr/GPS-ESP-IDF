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

static const char *TAG_GPS = "GPS";
static const char *TAG_RTK = "RTK";

/* ---------------------------- System config ---------------------------- */
#define GPS_TX GPIO_NUM_14 // ESP RX  <- GPS TX
#define GPS_RX GPIO_NUM_12 // ESP TX  -> GPS RX

#define GPS_UART_NUM       UART_NUM_1
#define GPS_BAUDRATE       115200
#define GPS_RX_CHUNK       128
#define GPS_BUFF_SIZE      4096
#define GPS_SAMPLE_RATE_MS 200 // 5 Hz

/* ---------------------------- Sample types ---------------------------- */
typedef struct {
    int32_t latitude;     // deg * 1e-7, + = N
    int32_t longitude;    // deg * 1e-7, + = E
    float   altitude;     // m above calibrated pad altitude
    float   vel_vertical; // m/s, up positive
    uint8_t sAcc;         // speed accuracy * 10 (m/s)
    uint8_t fix;          // gpsFix type, 0 if not OK
} gps_sample_t;

// RINEX file
typedef struct {
    uint8_t numSV;                // # of satellites following
    double  carrier_phase[32];    // L1 cycles
    double  pseudorange_meas[32]; // m
    float   doppler_meas[32];     // Hz
    uint8_t sv_id[32];            // Space Vehicle Number
    uint8_t meas_quality[32];     // Nav Measurements Quality Indicator
    int8_t  signal_strength[32];  // dbHz
    uint8_t lli[32];              // Loss of lock indicator (RINEX)
} rtk_sample_t;

static gps_sample_t gps_sample_g = {0};
static rtk_sample_t rtk_sample_g = {0};
static portMUX_TYPE xGPSMutex    = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t xTaskLog     = NULL;

#define GPS_BIT BIT(0)
#define RTK_BIT BIT(1)

static atomic_bool on_pad = true;

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

static bool streaming = false;

/* ---------------------------- GPS config ---------------------------- */
#define UBLOX6_ACTIVE   1
#define UBLOX7_ACTIVE   0
#define GPS_DATA_ACTIVE 0
#define RTK_DATA_ACTIVE 1

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

static struct {
    uint8_t   class;
    uint8_t   id;
    ubx_ack_t response;
} ack_state;

static struct {
    uint8_t  numSV;
    uint16_t pDOP; // x0.01
    bool     valid;
    char     swVersion[30];
    char     hwVersion[10];
} gps_info;

static struct {
    uint32_t itow;
    uint8_t  have;
    bool     fix_ok;
} epoch;

static struct {
    float    hmsl_m;
    uint32_t vacc_mm;
    float    mean;
    uint32_t cnt;
} alt_cal;

// {class, id, payload[len], len}
static const ubx_cmd_t gps_config[] = {
    // CFG-MSG: disable NMEA sentences (class 0xF0)
    {0x06, 0x01, {0xF0, 0x00, 0x00}, 3}, // GGA off
    {0x06, 0x01, {0xF0, 0x01, 0x00}, 3}, // GLL off
    {0x06, 0x01, {0xF0, 0x02, 0x00}, 3}, // GSA off
    {0x06, 0x01, {0xF0, 0x03, 0x00}, 3}, // GSV off
    {0x06, 0x01, {0xF0, 0x04, 0x00}, 3}, // RMC off
    {0x06, 0x01, {0xF0, 0x05, 0x00}, 3}, // VTG off

    // CFG-MSG: enable UBX messages
    {0x06, 0x01, {0x01, 0x02, 0x01}, 3}, // NAV-POSLLH on
    {0x06, 0x01, {0x01, 0x12, 0x01}, 3}, // NAV-VELNED on
    {0x06, 0x01, {0x01, 0x03, 0x01}, 3}, // NAV-STATUS on
    {0x06, 0x01, {0x01, 0x06, 0x05}, 3}, // NAV-SOL on, every 5th epoch

    // CFG-RATE: measRate, navRate 1, GPS time
    {0x06, 0x08, {GPS_SAMPLE_RATE_MS & 0xFF, GPS_SAMPLE_RATE_MS >> 8, 0x01, 0x00, 0x01, 0x00}, 6},

    // CFG-NAV5: dynModel only, Pedestrian (3)
    {0x06, 0x24, {0x01, 0x00, 0x03}, 36},

// CFG-NAV5: dynModel only, Airborne <4g (8) -- use this for actual flight
//{0x06, 0x24, {0x01, 0x00, 0x08}, 36},

/* RTK MESSAGES */
#if UBLOX6_ACTIVE
    // CFG-MSG: enable RXM messages
    {0x06, 0x01, {0x02, 0x10, 0x01}, 3}, // RXM-RAW on
    {0x06, 0x01, {0x02, 0x11, 0x01}, 3}, // RXM-SFRB on
#endif

#if UBLOX7_ACTIVE
    // CFG-MSG: enable TRK messages
    {0x06, 0x01, {0x03, 0x0A, 0x01}, 3}, // TRK-TRKD5 on
    {0x06, 0x01, {0x03, 0x0F, 0x01}, 3}, // TRK-SFRBX on

    // CFG-MSG: enable NAV messages
    {0x06, 0x01, {0x01, 0x22, 0x01}, 3}, // NAV-CLOCK on
    {0x06, 0x01, {0x01, 0x30, 0x01}, 3}, // NAV-SVINFO on
#endif
};

/* ---------------------------- RTK ---------------------------- */
#if UBLOX6_ACTIVE
static void UBLOX6_ACTIVE_enable_rxm(void) {
    const uint8_t rxm_raw[] = {0xB5, 0x62, 0x09, 0x01, 0x10, 0x00, 0xC8, 0x16, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x97, 0x69, 0x21, 0x00, 0x00, 0x00, 0x02, 0x10, 0x2B, 0x22};

    const uint8_t rxm_sfrb[] = {0xB5, 0x62, 0x09, 0x01, 0x10, 0x00, 0x0C, 0x19, 0x00, 0x00, 0x00, 0x00,
                                0x00, 0x00, 0x83, 0x69, 0x21, 0x00, 0x00, 0x00, 0x02, 0x11, 0x5F, 0xF0};

    uart_write_bytes(GPS_UART_NUM, (const char *)rxm_raw, sizeof(rxm_raw));
    vTaskDelay(pdMS_TO_TICKS(100));

    uart_write_bytes(GPS_UART_NUM, (const char *)rxm_sfrb, sizeof(rxm_sfrb));
    vTaskDelay(pdMS_TO_TICKS(100));
}
#endif

/* ---------------------------- Epoch / calibration ---------------------------- */
static inline bool altitude_calibration(void) {
    if (atomic_load(&on_pad) && alt_cal.cnt < CAL_MAX_SAMPLES && alt_cal.vacc_mm < CAL_VACC_MAX_MM) {
        alt_cal.cnt++;
        alt_cal.mean += (alt_cal.hmsl_m - alt_cal.mean) / alt_cal.cnt;
    }

    return alt_cal.cnt >= CAL_MIN_SAMPLES;
}

static inline void ubx_epoch_touch(uint32_t itow) {
    if (itow != epoch.itow) {
        epoch.itow   = itow;
        epoch.have   = 0;
        epoch.fix_ok = false;
    }
}

static inline void ubx_epoch_post(uint8_t bit, gps_sample_t *gps) {
    epoch.have |= bit;
    if (epoch.have != HAVE_ALL || !streaming)
        return;

    epoch.have = 0;

    if (!epoch.fix_ok) {
        portENTER_CRITICAL(&xGPSMutex);
        gps_sample_g.fix = 0;
        portEXIT_CRITICAL(&xGPSMutex);
        return;
    }

    if (!altitude_calibration())
        return;

    gps->altitude = alt_cal.hmsl_m - alt_cal.mean;

#if GPS_DATA_ACTIVE

    portENTER_CRITICAL(&xGPSMutex);
    gps_sample_g.latitude     = gps->latitude;
    gps_sample_g.longitude    = gps->longitude;
    gps_sample_g.altitude     = gps->altitude;
    gps_sample_g.vel_vertical = gps->vel_vertical;
    gps_sample_g.sAcc         = gps->sAcc;
    gps_sample_g.fix          = gps->fix;
    portEXIT_CRITICAL(&xGPSMutex);

#endif

    if (xTaskLog)
        xTaskNotify(xTaskLog, GPS_BIT, eSetBits);
}
/* ---------------------------- UBX parsing ---------------------------- */
static inline void ubx_checksum(const uint8_t *frame, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    uint8_t a_sum = 0, b_sum = 0;

    for (int i = 0; i < len; i++) {
        a_sum += frame[i];
        b_sum += a_sum;
    }

    *ck_a = a_sum;
    *ck_b = b_sum;
}

static void ubx_handle_message(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len, gps_sample_t *gps) {
    /* ACK / NAK */
    if (class == 0x05 && len == 2 && (id == 0x01 || id == 0x00)) {
        if (payload[0] == ack_state.class && payload[1] == ack_state.id)
            ack_state.response = (id == 0x01) ? ACK_OK : ACK_NAK;

        return;
    }

    /* MON-VER */
    if (class == 0x0A && id == 0x04 && len >= 40) {
        memcpy(gps_info.swVersion, &payload[0], 30);
        memcpy(gps_info.hwVersion, &payload[30], 10);

        ESP_LOGI(TAG_GPS, "Software: %s, Hardware: %s", gps_info.swVersion, gps_info.hwVersion);
        return;
    }

    /* NAV-POSLLH */
    if (class == 0x01 && id == 0x02 && len == 28) {
        uint32_t iTOW; // GPS Millisecond Time of Week (ms)
        int32_t  lon;  // Longitude (deg * 1e-7)
        int32_t  lat;  // Latitude (deg * 1e-7)
        int32_t  hMSL; // Height above mean sea level (mm)
        uint32_t vAcc; // Vertical Accuracy Estimate (mm)

        memcpy(&iTOW, &payload[0], sizeof(iTOW));
        memcpy(&lon, &payload[4], sizeof(lon));
        memcpy(&lat, &payload[8], sizeof(lat));
        memcpy(&hMSL, &payload[16], sizeof(hMSL));
        memcpy(&vAcc, &payload[24], sizeof(vAcc));

        alt_cal.hmsl_m  = hMSL * 0.001f;
        alt_cal.vacc_mm = vAcc;

        gps->longitude = lon; // + = East
        gps->latitude  = lat; // + = North

        ubx_epoch_touch(iTOW);
        ubx_epoch_post(UBX_POSLLH, gps);

        return;
    }

    /* NAV-VELNED */
    if (class == 0x01 && id == 0x12 && len == 36) {
        uint32_t iTOW; // GPS Millisecond Time of Week (ms)
        int32_t  velD; // NED down velocity (cm/s)
        uint32_t sAcc; // Speed Accuracy Estimate (cm/s)

        memcpy(&iTOW, &payload[0], sizeof(iTOW));
        memcpy(&velD, &payload[12], sizeof(velD));
        memcpy(&sAcc, &payload[28], sizeof(sAcc));

        if (sAcc > 2550) // Constraint for uint8_t storage (25.5 m/s * 10)
            sAcc = 2550;

        gps->vel_vertical = -(velD * 0.01f);              // m/s (up positive)
        gps->sAcc         = (uint8_t)roundf(sAcc * 0.1f); // m/s * 10

        ubx_epoch_touch(iTOW);
        ubx_epoch_post(UBX_VELNED, gps);

        return;
    }

    /* NAV-STATUS */
    if (class == 0x01 && id == 0x03 && len == 16) {
        uint32_t iTOW;                // GPS Millisecond Time of Week (ms)
        uint8_t  gpsFix = payload[4]; // GPSfix Type
        uint8_t  flags  = payload[5]; // Navigation Status Flags

        memcpy(&iTOW, &payload[0], sizeof(iTOW));

        bool fixOk = (flags & 0x01) != 0; // gpsFixOk
        gps->fix   = fixOk ? gpsFix : 0;

        ubx_epoch_touch(iTOW);
        epoch.fix_ok = fixOk && gpsFix == 3;
        ubx_epoch_post(UBX_STATUS, gps);

        return;
    }

    /* NAV-SOL */
    if (class == 0x01 && id == 0x06 && len == 52) {
        uint16_t pDOP;  // Position DOP
        uint8_t  numSV; // Number of SVs used in Nav Solution

        memcpy(&pDOP, &payload[44], sizeof(pDOP));
        numSV = payload[47];

        gps_info.numSV = numSV;
        gps_info.pDOP  = pDOP;
        gps_info.valid = true;

        return;
    }

    /* RXM-RAW */
    if (class == 0x02 && id == 0x10 && len >= 32) {
        uint32_t iTOW;
        int16_t  week;
        uint8_t  numSV = payload[6];

        memcpy(&iTOW, &payload[0], 4);
        memcpy(&week, &payload[4], 2);

        rtk_sample_g.numSV = numSV;

        // 8 byte header + 24 byte/satellite
        if (len != 8 + (uint16_t)numSV * 24)
            return;

        for (int i = 0; i < numSV; i++) {
            // Current satellite data
            const uint8_t *sv = &payload[8 + i * 24];

            double cpMes; // Carrier phase measurement (L1 cycles)
            double prMes; // Pseudorange measurement (m)
            float  doMes; // Doppler measurement (Hz)

            memcpy(&cpMes, &sv[0], 8);
            memcpy(&prMes, &sv[8], 8);
            memcpy(&doMes, &sv[16], 4);

            uint8_t sv_id = sv[20]; // Space Vehicle Number
            uint8_t mesQI = sv[21]; // Nav Measurements Quality Indicator
            int8_t  cno   = sv[22]; // Signal strength C/No (dbHz)
            uint8_t lli   = sv[23]; // Loss of lock indicator (RINEX)

            if (mesQI < 5)
                ESP_LOGW(TAG_RTK, "Invalid Pseudorange value");

            rtk_sample_g.carrier_phase[i]    = cpMes;
            rtk_sample_g.pseudorange_meas[i] = prMes;
            rtk_sample_g.doppler_meas[i]     = doMes;
            rtk_sample_g.sv_id[i]            = sv_id;
            rtk_sample_g.meas_quality[i]     = mesQI;
            rtk_sample_g.signal_strength[i]  = cno;
            rtk_sample_g.lli[i]              = lli;

            xTaskNotify(xTaskLog, RTK_BIT, eSetBits);

            // add the rest if necessary for RINEX
        }

        return;
    }

    /* RXM-SFRB */
    if (class == 0x02 && id == 0x11 && len == 42) {

        return;
    }
}

static void ubx_parse_message(uint8_t c, gps_sample_t *gps) {
    static enum { SYNC, HEADER, BODY } state = SYNC;

    static uint16_t sync;
    static uint8_t  frame[UBX_HDR_LEN + UBX_MAX_PAYLOAD + UBX_CK_LEN];
    static uint16_t len, ubx_pos;
    static uint8_t  checksum[2];

    switch (state) {
    case SYNC:
        sync = (sync << 8) | c;
        if (sync == (UBX_SYNC1 << 8 | UBX_SYNC2)) {
            ubx_pos = 0;
            state   = HEADER;
        }
        break;

    case HEADER:
        frame[ubx_pos++] = c;
        if (ubx_pos == UBX_HDR_LEN) {
            len = frame[2] | (frame[3] << 8);
            if (len > UBX_MAX_PAYLOAD) {
                state = SYNC;
                break;
            }
            state = BODY;
        }
        break;

    case BODY:
        frame[ubx_pos++] = c;
        if (ubx_pos == UBX_HDR_LEN + len + UBX_CK_LEN) {
            ubx_checksum(frame, UBX_HDR_LEN + len, &checksum[0], &checksum[1]);
            if (checksum[0] == frame[ubx_pos - 2] && checksum[1] == frame[ubx_pos - 1])
                ubx_handle_message(frame[0], frame[1], &frame[UBX_HDR_LEN], len, gps);
            state = SYNC;
        }
        break;
    }
}

/* ---------------------------- UBX TX ---------------------------- */
static inline void ubx_send(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len) {
    if (len > UBX_MAX_PAYLOAD)
        return;

    uint8_t frame[2 + UBX_HDR_LEN + UBX_MAX_PAYLOAD + UBX_CK_LEN];

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = class;
    frame[3] = id;
    frame[4] = len & 0xFF;
    frame[5] = len >> 8;
    if (len)
        memcpy(&frame[6], payload, len);
    ubx_checksum(&frame[2], UBX_HDR_LEN + len, &frame[6 + len], &frame[7 + len]);
    uart_write_bytes(GPS_UART_NUM, frame, 8 + len);
}

static bool ubx_send_wait_ack(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len, int idx) {
    gps_sample_t dummy_gps = {0};
    uint8_t      buf[GPS_RX_CHUNK];

    for (int attempt = 0; attempt < UBX_ACK_RETRIES; attempt++) {
        ack_state.class    = class;
        ack_state.id       = id;
        ack_state.response = ACK_NONE;

        ubx_send(class, id, payload, len);

        TickType_t start = xTaskGetTickCount();
        while (ack_state.response == ACK_NONE && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(UBX_ACK_TIMEOUT_MS)) {
            int n = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
            for (int i = 0; i < n; i++)
                ubx_parse_message(buf[i], &dummy_gps);
        }

        if (ack_state.response == ACK_OK) {
            ESP_LOGD(TAG_GPS, "ACK 0x%02X 0x%02X after %lu ms", class, id,
                     (unsigned long)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS));
            return true;
        }

        if (ack_state.response == ACK_NAK) {
            ESP_LOGE(TAG_GPS, "NAK for 0x%02X 0x%02X - [CFG:%d]", class, id, idx);
            return false;
        }

        ESP_LOGW(TAG_GPS, "No ACK for 0x%02X 0x%02X - [CFG:%d] (attempt %d/%d)", class, id, idx, attempt + 1,
                 UBX_ACK_RETRIES);
    }

    return false;
}

static bool gps_set_baudrate(uint32_t baudrate) {
    uint8_t payload[20] = {
        0x01, 0x00,             // portID = UART1, reserved
        0x00, 0x00,             // txReady
        0xD0, 0x08, 0x00, 0x00, // mode = 8N1
        0x00, 0xC2, 0x01, 0x00, // baudrate
        0x07, 0x00,             // inProtoMask: UBX + NMEA
        0x03, 0x00,             // outProtoMask: UBX + NMEA
        0x00, 0x00,             // flags
        0x00, 0x00              // reserved
    };

    // Update baudrate (offsets 8-11)
    payload[8]  = (uint8_t)(baudrate);
    payload[9]  = (uint8_t)(baudrate >> 8);
    payload[10] = (uint8_t)(baudrate >> 16);
    payload[11] = (uint8_t)(baudrate >> 24);

    ubx_send(0x06, 0x00, payload, sizeof(payload));

    // Ensure the complete command leaves the ESP32 UART
    ESP_ERROR_CHECK(uart_wait_tx_done(GPS_UART_NUM, pdMS_TO_TICKS(200)));

    return true;
}

/* ---------------------------- GPS task ---------------------------- */
static bool gps_configure(void) {
    bool ok = true;
    for (int i = 0; i < sizeof(gps_config) / sizeof(gps_config[0]); i++) {
        const ubx_cmd_t *c = &gps_config[i];
        if (!ubx_send_wait_ack(c->class, c->id, c->payload, c->len, i)) {
            ESP_LOGE(TAG_GPS, "Config %lu failed (0x%02X 0x%02X)", (unsigned long)i, c->class, c->id);
            ok = false;
        }
    }
    return ok;
}

static void gps_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_RX, GPS_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUFF_SIZE, 0, 0, NULL, 0));

    ESP_LOGI(TAG_GPS, "GPS UART initialized (baud %d)", 9600);

    gps_set_baudrate(GPS_BAUDRATE);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(uart_set_baudrate(GPS_UART_NUM, GPS_BAUDRATE));
    uart_flush_input(GPS_UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG_GPS, "GPS UART update (baud %d)", GPS_BAUDRATE);

// RTK
#if UBLOX6_ACTIVE
    UBLOX6_ACTIVE_enable_rxm();
#endif

    if (!gps_configure())
        ESP_LOGE(TAG_GPS, "GPS configuration failed");
    else
        ESP_LOGI(TAG_GPS, "GPS configuration succeeded");

    epoch     = (typeof(epoch)){0};
    alt_cal   = (typeof(alt_cal)){0};
    streaming = true;
}

static void task_gps(void *pvParameters) {
    gps_init();

    uint8_t      rx_buffer[GPS_RX_CHUNK];
    gps_sample_t gps = {0};

    ubx_send(0x0A, 0x04, NULL, 0); // MON-VER poll

    TickType_t last_log = 0;
    while (true) {
        int rx_len = uart_read_bytes(GPS_UART_NUM, rx_buffer, GPS_RX_CHUNK, pdMS_TO_TICKS(20));
        for (int i = 0; i < rx_len; ++i)
            ubx_parse_message(rx_buffer[i], &gps);

        if (gps_info.valid && xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG_GPS, "sats=%u pdop=%.2f fix=%u vAcc=%lu cal=%lu/%d", gps_info.numSV, gps_info.pDOP * 0.01f,
                     gps_sample_g.fix, (unsigned long)alt_cal.vacc_mm, (unsigned long)alt_cal.cnt, CAL_MIN_SAMPLES);
            last_log = xTaskGetTickCount();
        }
    }
}

/* ---------------------------- Log task ---------------------------- */
static void task_log(void *pvParameters) {
    uint32_t notified;
    while (true) {
        xTaskNotifyWait(0, UINT32_MAX, &notified, portMAX_DELAY);

#if GPS_DATA_ACTIVE
        gps_sample_t s;
        portENTER_CRITICAL(&xGPSMutex);
        s = gps_sample_g;
        portEXIT_CRITICAL(&xGPSMutex);

        ESP_LOGI("SAMPLE", "fix=%u lat=%.7f lon=%.7f alt=%.2f m vz=%.2f m/s sAcc=%.1f m/s", s.fix, s.latitude / 1e7,
                 s.longitude / 1e7, s.altitude, s.vel_vertical, s.sAcc * 0.1f);
#endif

#if RTK_DATA_ACTIVE
        rtk_sample_t s;
        portENTER_CRITICAL(&xGPSMutex);
        s = rtk_sample_g;
        portEXIT_CRITICAL(&xGPSMutex);

        for (int i = 0; i < s.numSV; i++) {
            ESP_LOGI(TAG_RTK, "Satellite - %d/%d (%u): cp=%.4lf pr=%.4lf m do=%.2f | qi=%d ss=%d lli=%u", i + 1,
                     s.numSV, s.sv_id[i], s.carrier_phase[i], s.pseudorange_meas[i], s.doppler_meas[i],
                     s.meas_quality[i], s.signal_strength[i], s.lli[i]);
        }
#endif
    }
}

void app_main(void) {
    xTaskCreatePinnedToCore(task_log, "LOG", 4096, NULL, 5, &xTaskLog, 1);
    xTaskCreatePinnedToCore(task_gps, "GPS", 4096, NULL, 5, NULL, 1);
}