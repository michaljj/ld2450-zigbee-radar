#ifndef ZIGBEE_HANDLER_H
#define ZIGBEE_HANDLER_H

#include "esp_zigbee_core.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false                                /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN        /* aging timeout of device */
#define ED_KEEP_ALIVE                   3000                                 /* 3000 millisecond */
#define HA_ESP_ENDPOINT                 10                                   /* esp light bulb device endpoint, used to process light controlling commands */
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define DEFAULT_RANGE                   300
#define DEFAULT_TIMEOUT                 20
#define DEFAULT_UPDATE_RATE             20
#define DEFAULT_ANGLE                   0

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME           "\x05""JUREK"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER            "\x03""JP0" /* Customized model identifier */

#define RADAR_CONFIG_CLUSTER_ID            0Xff00


typedef struct esp_zb_radar_cfg_s
{
    esp_zb_basic_cluster_cfg_t basic_cfg;
    esp_zb_identify_cluster_cfg_t identify_cfg;
    esp_zb_occupancy_sensing_cluster_cfg_t occupancy_sensing_cfg;
} esp_zb_radar_cfg_t;

typedef struct radar_config_attr_s
{
    const uint16_t range_attr_id;
    uint16_t range_attr_val;
    const uint16_t update_rate_attr_id;
    uint8_t update_rate_attr_val;
    const uint16_t timeout_attr_id;
    uint16_t timeout_attr_val;
    const uint16_t angle_attr_id;
    uint16_t angle_attr_val;
}radar_config_attr_t;


void zigbeeHandler_Init(void);
void zigbeeHandler_radarUpdate(uint8_t);

#endif // ZIGBEE_HANDLER_H