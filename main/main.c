#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "LD2450_driver.h"
#include "Zigbee_handler.h"
#include "nvs_flash.h"

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    printf("Hello world!\n");
    LD2450_Init();
    zigbeeHandler_Init();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        zigbeeHandler_radarUpdate(LD2450_GetOccupancy());
    }
}
