#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "LD2450_driver.h"

static QueueHandle_t uart_queue;
static const char *TAG = "LD2450";

/* --------------------STATIC CODE-------------------- */
// Forward declarations for internal helpers
static void uart_init(void);
static void enter_config_mode(void);
static void exit_config_mode(void);
static void get_firmware_version(void);
static void reboot_sensor(void);
//static void factory_reset_sensor(void);
static void process_uart_data(const uint8_t *data, size_t len);
static void process_target_data(const uint8_t *bytes);
static void watchdog_task(void *pvParameters);
static void uart_event_task(void *pvParameters);

// Local sensor state
static LD2450_state_t sensor_state = {0};

// Command bytes
static const uint8_t CMD_ENTER_CONFIG[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
static const uint8_t CMD_ENTER_CONFIG_ACK_OK[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x01, 0x00, 0x40, 0x00, 0x04, 0x03, 0x02, 0x01};

static const uint8_t CMD_EXIT_CONFIG[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
static const uint8_t CMD_EXIT_CONFIG_ACK_OK[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFE, 0x01, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};

static const uint8_t CMD_GET_FIRMWARE[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
static const uint8_t CMD_REBOOT[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA3, 0x00, 0x04, 0x03, 0x02, 0x01};


static void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = LD2450_BAUD_RATE,
        .data_bits = LD2450_DATA_BITS,
        .parity = LD2450_PARITY,
        .stop_bits = LD2450_STOP_BITS,
        .flow_ctrl = LD2450_FLOWCTRL,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LD2450_UART_NUM, UART_BUFF_SIZE * 2, UART_BUFF_SIZE * 2, 20, &uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(LD2450_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LD2450_UART_NUM, LD2450_UART_TX_IO, LD2450_UART_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void enter_config_mode(void) {
    uint8_t response[18];
    sensor_state.config_mode = true;
    uart_write_bytes(LD2450_UART_NUM, (const char *)CMD_ENTER_CONFIG, sizeof(CMD_ENTER_CONFIG));
    uart_wait_tx_done(LD2450_UART_NUM, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_read_bytes(LD2450_UART_NUM, response, sizeof(CMD_ENTER_CONFIG_ACK_OK), pdMS_TO_TICKS(1000));
    if(0 == memcmp(response, CMD_ENTER_CONFIG_ACK_OK, sizeof(CMD_ENTER_CONFIG_ACK_OK)))
    {
        ESP_LOGI(TAG, "Entered config mode");
    }else{
        ESP_LOGE(TAG, "Failed to enter config mode");
        sensor_state.config_mode = false;
    }
}

static void exit_config_mode(void) {
    uint8_t response[18];

    uart_write_bytes(LD2450_UART_NUM, (const char *)CMD_EXIT_CONFIG, sizeof(CMD_EXIT_CONFIG));
    uart_wait_tx_done(LD2450_UART_NUM, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_read_bytes(LD2450_UART_NUM, response, sizeof(CMD_EXIT_CONFIG_ACK_OK), pdMS_TO_TICKS(500));
    if(0 == memcmp(response, CMD_EXIT_CONFIG_ACK_OK, sizeof(CMD_EXIT_CONFIG_ACK_OK)))
    {
        sensor_state.config_mode = false;
        ESP_LOGI(TAG, "Exited config mode");
    }else
    {
        ESP_LOGE(TAG, "Failed to exit config mode");
    }
}

static void get_firmware_version(void) {
    enter_config_mode();

    uart_write_bytes(LD2450_UART_NUM, (const char *)CMD_GET_FIRMWARE, sizeof(CMD_GET_FIRMWARE));
    uart_wait_tx_done(LD2450_UART_NUM, pdMS_TO_TICKS(1000));

    uint8_t response[22];
    int len = uart_read_bytes(LD2450_UART_NUM, response, sizeof(response), pdMS_TO_TICKS(2000));

    if (len >= 14) {
        int firmware_major = response[13];
        int firmware_minor = response[12];
        snprintf(sensor_state.firmware_version, sizeof(sensor_state.firmware_version), 
                 "V%d.%02d", firmware_major, firmware_minor);
        ESP_LOGI(TAG, "Firmware: %s", sensor_state.firmware_version);
    } else {
        strcpy(sensor_state.firmware_version, "Unknown");
        ESP_LOGW(TAG, "Failed to read firmware version");
    }

    exit_config_mode();
}

static void reboot_sensor(void) {
    enter_config_mode();
    uart_write_bytes(LD2450_UART_NUM, (const char *)CMD_REBOOT, sizeof(CMD_REBOOT));
    uart_wait_tx_done(LD2450_UART_NUM, pdMS_TO_TICKS(1000));
    vTaskDelay(pdMS_TO_TICKS(2000));
    sensor_state.config_mode = false;
    ESP_LOGI(TAG, "Sensor rebooted");
}




static void process_target_data(const uint8_t *bytes) {
    int64_t now = esp_timer_get_time();

    // Check update interval throttling
    if ((now - sensor_state.last_update_time) < (sensor_state.config.update_interval_ms * 1000)) {
        return;
    }
    sensor_state.last_update_time = now;

    float max_distance = sensor_state.config.max_distance_cm * 10.0f; // Convert to mm
    float installation_angle = sensor_state.config.installation_angle_deg * DEGREES_TO_RADIANS;

    bool any_target_detected = false;

    // Process each target
    for (int t = 0; t < LD2450_MAX_TARGETS; t++) {
        int offset = 4 + (t * 8);
        uint16_t resolution = 0;
        memcpy(&resolution, &bytes[offset + 6], sizeof(resolution));
        bool detected = resolution != 0;

        if (detected) {
            // Extract coordinates (use memcpy to avoid unaligned access)
            int16_t x = 0;
            memcpy(&x, &bytes[offset], sizeof(x));
            if (x < 0) x += MIN_INT16_VAL;
            else x = -x;

            int16_t y = 0;
            memcpy(&y, &bytes[offset + 2], sizeof(y));
            if (y < 0) y += MIN_INT16_VAL;
            else y = -y;

            // Calculate distance
            float distance = sqrtf(x * x + y * y);

            // Apply inverse mounting
            if (sensor_state.config.inverse_mounting) {
                x = -x;
            }

            // Check distance filter
            if (distance > max_distance) {
                detected = false;
            } else {
                // Calculate angle
                float angle = atan2f(y, x);

                // Apply installation angle rotation
                if (installation_angle != 0) {
                    float rotated_angle = angle - installation_angle;
                    x = (int16_t)(distance * cosf(rotated_angle));
                    y = (int16_t)(distance * sinf(rotated_angle));
                }

                // Extract speed
                int16_t speed = 0;
                memcpy(&speed, &bytes[offset + 4], sizeof(speed));
                if (speed < 0) speed += MIN_INT16_VAL;
                else speed = -speed;

                // Update target state
                sensor_state.targets[t].active = true;
                sensor_state.targets[t].x = x;
                sensor_state.targets[t].y = y;
                sensor_state.targets[t].speed = speed;
                sensor_state.targets[t].resolution = resolution;
                sensor_state.targets[t].distance = distance;
                sensor_state.targets[t].angle = (angle * RADIANS_TO_DEGREES) - 90.0f;
                sensor_state.targets[t].last_update = now;

                any_target_detected = true;
            }
        }

        if (!detected) {
            sensor_state.targets[t].active = false;
            sensor_state.targets[t].x = 0;
            sensor_state.targets[t].y = 0;
            sensor_state.targets[t].speed = 0;
            sensor_state.targets[t].resolution = 0;
            sensor_state.targets[t].distance = 0;
            sensor_state.targets[t].angle = 0;
        }
    }

    sensor_state.overall_occupancy = any_target_detected;
}



static void process_uart_data(const uint8_t *data, size_t len) {
    static uint8_t frame_buffer[FRAME_SIZE];
    static size_t frame_pos = 0;

    for (size_t i = 0; i < len; i++) {
        frame_buffer[frame_pos++] = data[i];

        // Check for frame delimiter
        if (frame_pos >= 2 && 
            frame_buffer[frame_pos - 2] == FRAME_DELIMITER_1 && 
            frame_buffer[frame_pos - 1] == FRAME_DELIMITER_2) {

            if (frame_pos == FRAME_SIZE) {
                process_target_data(frame_buffer);
                //ESP_LOGW(TAG, "valid frame received, printing raw data");
                //LD2450_PrintRawFrame(frame_buffer, FRAME_SIZE);
            } else {
                ESP_LOGW(TAG, "Expected %d bytes but received %d", FRAME_SIZE, frame_pos);
            }
            frame_pos = 0;
        }

        // Prevent buffer overflow
        if (frame_pos >= FRAME_SIZE) {
            frame_pos = 0;
        }
    }
}


static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(UART_BUFF_SIZE);

    while (1) {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    if(sensor_state.config_mode == true) break;
                    int len = uart_read_bytes(LD2450_UART_NUM, data, event.size, portMAX_DELAY);
                    if (len > 0) {
                        process_uart_data(data, len);
                    }
                    break;
                    
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO overflow");
                    uart_flush_input(LD2450_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART buffer full");
                    uart_flush_input(LD2450_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                default:
                    break;
            }
        }
    }

    free(data);
    vTaskDelete(NULL);
}

static void watchdog_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (sensor_state.config.aggressive_target_clearing) {
            int64_t now = esp_timer_get_time();
            int64_t timeout_us = sensor_state.config.aggressive_timeout_ms * 1000;

            if (sensor_state.targets[0].active) {
                if ((now - sensor_state.targets[0].last_update) > timeout_us) {
                    ESP_LOGD(TAG, "No update in %d ms, clearing and rebooting", 
                             sensor_state.config.aggressive_timeout_ms);
                    reboot_sensor();
                }
            }
        }
    }
}

// Initialize sensor with default configuration
void LD2450_Init(void) {
    // Default configuration
    sensor_state.config.max_distance_cm = 600.0f;
    sensor_state.config.installation_angle_deg = 0.0f;
    sensor_state.config.inverse_mounting = false;
    sensor_state.config.aggressive_target_clearing = false;
    sensor_state.config.aggressive_timeout_ms = 3000;
    sensor_state.config.update_interval_ms = 300;

    // Initialize UART
    uart_init();

    // Get firmware version
    vTaskDelay(pdMS_TO_TICKS(2000));
    get_firmware_version();

    // Start tasks
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);
    xTaskCreate(watchdog_task, "watchdog_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "LD2450 sensor initialized");
}

LD2450_state_t* LD2450_GetState(void) {
    return &sensor_state;
}

void LD2450_PrintState(void) {
    printf("\n=== LD2450 Sensor State ===\n");
    printf("Firmware: %s\n", sensor_state.firmware_version);
    printf("Overall Occupancy: %s\n", sensor_state.overall_occupancy ? "YES" : "NO");
    
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        if (sensor_state.targets[i].active) {
            printf("Target %d: X=%d Y=%d Speed=%.2f Dist=%.0f Angle=%.1f\n",
                   i + 1,
                   sensor_state.targets[i].x,
                   sensor_state.targets[i].y,
                   sensor_state.targets[i].speed / 100.0f,
                   sensor_state.targets[i].distance,
                   sensor_state.targets[i].angle);
        }
    }
}

uint8_t LD2450_GetOccupancy()
{
    return sensor_state.overall_occupancy;
}
