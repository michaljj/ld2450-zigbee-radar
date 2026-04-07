#ifndef LD2450_DRIVER_H
#define LD2450_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

// Keep existing platform defines (may be overridden in CMake or sdkconfig)
#define LD2450_UART_NUM     UART_NUM_1
#define LD2450_UART_TX_IO   22
#define LD2450_UART_RX_IO   23
#define LD2450_BAUD_RATE    256000
#define LD2450_DATA_BITS    UART_DATA_8_BITS
#define LD2450_PARITY       UART_PARITY_DISABLE
#define LD2450_STOP_BITS    UART_STOP_BITS_1
#define LD2450_FLOWCTRL     UART_HW_FLOWCTRL_DISABLE

#define UART_BUFF_SIZE      1024

#define LD2450_MAX_TARGETS  3

#define FRAME_SIZE            30
#define FRAME_DELIMITER_1     0x55
#define FRAME_DELIMITER_2     0xCC

#define MIN_INT16_VAL         (-32768)
#define RADIANS_TO_DEGREES    (180.0f / M_PI)
#define DEGREES_TO_RADIANS    (M_PI / 180.0f)

typedef struct {
    float max_distance_cm;
    float installation_angle_deg;
    bool inverse_mounting;
    bool aggressive_target_clearing;
    uint32_t aggressive_timeout_ms;
    uint32_t off_delay_ms;
    uint32_t update_interval_ms;
} LD2450_config_t;

typedef struct {
    bool active;
    int16_t x;
    int16_t y;
    int16_t speed;
    uint16_t resolution;
    float distance;
    float angle;
    int64_t last_update;
} LD2450_target_t;

typedef struct {
    LD2450_config_t config;
    LD2450_target_t targets[LD2450_MAX_TARGETS];
    bool overall_occupancy;
    int64_t last_update_time;
    bool config_mode;
    char firmware_version[32];
} LD2450_state_t;

// Public API
void LD2450_Init(void);
LD2450_state_t* LD2450_GetState(void);
void LD2450_PrintState(void);
uint8_t LD2450_GetOccupancy();

#endif // LD2450_DRIVER_H
