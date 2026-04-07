#include "Zigbee_handler.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------GLOBAL VARIABLES-------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------LOCAL VARIABLES--------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/

static const char *TAG = "ZIGBEE_HANDLER";

static radar_config_attr_t radar_conf_attr = {
    .range_attr_id = 0x0000,
    .range_attr_val = DEFAULT_RANGE,
    .timeout_attr_id = 0x0001,
    .timeout_attr_val = DEFAULT_TIMEOUT,
    .update_rate_attr_id = 0x0002,
    .update_rate_attr_val = DEFAULT_UPDATE_RATE,
    .angle_attr_id = 0x0003,
    .angle_attr_val = DEFAULT_ANGLE,
};

static esp_zb_radar_cfg_t radar_cfg = {
    .basic_cfg =
        {
            .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
        },
    .identify_cfg = {
        .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
    },
    .occupancy_sensing_cfg = {
        .occupancy = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED,
        .sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_PIR,
    },
};

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------LOCAL FUNCTIONS--------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*-------------------------------------------------------------------DECLARATIONS-----------------------------------------------------------------------*/

static esp_zb_cluster_list_t *custom_radar_sensor_clusters_create(esp_zb_radar_cfg_t *radar_sensor);
static esp_zb_ep_list_t *custom_radar_sensor_ep_create(esp_zb_radar_cfg_t *radar_sensor);
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static int zb_zcl_radar_config_cluster_check_value_handler(uint16_t attr_id, uint8_t endpoint, uint8_t *value);
static void zb_zcl_radar_config_cluster_write_attr_handler(uint8_t endpoint, uint16_t attr_id, uint8_t *new_value, uint16_t manuf_code);
static void zb_handler_startup_attr_report(void);
static void esp_zb_task(void *pvParameters);


/*-------------------------------------------------------------------DEFINITIONS------------------------------------------------------------------------*/

static esp_zb_cluster_list_t *custom_radar_sensor_clusters_create(esp_zb_radar_cfg_t *radar_sensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(radar_sensor->basic_cfg));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ESP_MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ESP_MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(radar_sensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, esp_zb_occupancy_sensing_cluster_create(&(radar_sensor->occupancy_sensing_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    esp_zb_attribute_list_t *config_cluster = esp_zb_zcl_attr_list_create(RADAR_CONFIG_CLUSTER_ID);
    esp_zb_custom_cluster_add_custom_attr(config_cluster, radar_conf_attr.range_attr_id, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &(radar_conf_attr.range_attr_val));
    esp_zb_custom_cluster_add_custom_attr(config_cluster, radar_conf_attr.timeout_attr_id, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &(radar_conf_attr.timeout_attr_val));
    esp_zb_custom_cluster_add_custom_attr(config_cluster, radar_conf_attr.update_rate_attr_id, ESP_ZB_ZCL_ATTR_TYPE_U8,
                                      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &(radar_conf_attr.update_rate_attr_val));
    esp_zb_custom_cluster_add_custom_attr(config_cluster, radar_conf_attr.angle_attr_id, ESP_ZB_ZCL_ATTR_TYPE_U16,
                                      ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &(radar_conf_attr.angle_attr_val));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, config_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}


static esp_zb_ep_list_t *custom_radar_sensor_ep_create(esp_zb_radar_cfg_t *radar_sensor)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0};
    esp_zb_ep_list_add_ep(ep_list, custom_radar_sensor_clusters_create(radar_sensor), endpoint_config);
    return ep_list;
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static int zb_zcl_radar_config_cluster_check_value_handler(uint16_t attr_id, uint8_t endpoint, uint8_t *value)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Checking custom cluster attribute validity - Attr ID: 0x%04x, Endpoint: %d", attr_id, endpoint);
    
    switch (attr_id) {
        case 0x0000: { // range (U16)
            uint16_t range_val = *(uint16_t *)value;
            ESP_LOGI(TAG, "Range value to validate: %u", range_val);
            
            // Add your validation logic here
            if (range_val < 10 || range_val > 10000) {
                ESP_LOGW(TAG, "Range value out of bounds: %u (valid: 10-10000)", range_val);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }
        
        case 0x0001: { // timeout (U16)
            uint16_t timeout_val = *(uint16_t *)value;
            ESP_LOGI(TAG, "Timeout value to validate: %u", timeout_val);
            
            // Add your validation logic here
            if (timeout_val < 1 || timeout_val > 3600) {
                ESP_LOGW(TAG, "Timeout value out of bounds: %u (valid: 1-3600)", timeout_val);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }
        
        case 0x0002: { // update_rate (U8)
            uint8_t rate_val = *value;
            ESP_LOGI(TAG, "Update rate value to validate: %u", rate_val);
            
            // Add your validation logic here
            if (rate_val < 1 || rate_val > 100) {
                ESP_LOGW(TAG, "Update rate out of bounds: %u (valid: 1-100)", rate_val);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }
        
        case 0x0003: { // angle (U16)
            uint16_t angle_val = *(uint16_t *)value;
            ESP_LOGI(TAG, "Angle value to validate: %u", angle_val);
            
            // Add your validation logic here
            if (angle_val > 180) {
                ESP_LOGW(TAG, "Angle value out of bounds: %u (valid: 0-180)", angle_val);
                ret = ESP_ERR_INVALID_ARG;
            }
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown attribute ID: 0x%04x", attr_id);
            ret = ESP_ERR_NOT_FOUND;
            break;
    }
    
    return ret;
}

static void zb_zcl_radar_config_cluster_write_attr_handler(uint8_t endpoint, uint16_t attr_id, uint8_t *new_value, uint16_t manuf_code)
{
    ESP_LOGI(TAG, "Writing custom cluster attribute - Attr ID: 0x%04x, Endpoint: %d, Manuf: 0x%04x", 
             attr_id, endpoint, manuf_code);
    
    switch (attr_id) {
        case 0x0000: { // range (U16)
            uint16_t range_val = *(uint16_t *)new_value;
            ESP_LOGI(TAG, "New range value: %u", range_val);
            
            // Update your radar configuration here
            radar_conf_attr.range_attr_val = range_val;
            
            // Apply the new range to your radar hardware/driver
            // Example: radar_set_range(range_val);
            
            break;
        }
        
        case 0x0001: { // timeout (U16)
            uint16_t timeout_val = *(uint16_t *)new_value;
            ESP_LOGI(TAG, "New timeout value: %u", timeout_val);
            
            // Update your radar configuration here
            radar_conf_attr.timeout_attr_val = timeout_val;
            
            // Apply the new timeout to your radar hardware/driver
            // Example: radar_set_timeout(timeout_val);
            
            break;
        }
        
        case 0x0002: { // update_rate (U8)
            uint8_t rate_val = *new_value;
            ESP_LOGI(TAG, "New update rate value: %u", rate_val);
            
            // Update your radar configuration here
            radar_conf_attr.update_rate_attr_val = rate_val;
            
            // Apply the new update rate to your radar hardware/driver
            // Example: radar_set_update_rate(rate_val);
            
            break;
        }
        
        case 0x0003: { // angle (U16)
            uint16_t angle_val = *(uint16_t *)new_value;
            ESP_LOGI(TAG, "New angle value: %u", angle_val);
            
            // Update your radar configuration here
            radar_conf_attr.angle_attr_val = angle_val;
            
            // Apply the new angle to your radar hardware/driver
            // Example: radar_set_angle(angle_val);
            
            break;
        }
        
        default:
            ESP_LOGW(TAG, "Unknown attribute ID: 0x%04x", attr_id);
            break;
    }
}

void zb_handler_startup_attr_report(void)
{
    esp_zb_zcl_set_attribute_val(
    HA_ESP_ENDPOINT,
    RADAR_CONFIG_CLUSTER_ID,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    radar_conf_attr.range_attr_id,
    &(radar_conf_attr.range_attr_val),
    false  // Don't check for access control
    );
    esp_zb_zcl_set_attribute_val(
    HA_ESP_ENDPOINT,
    RADAR_CONFIG_CLUSTER_ID,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    radar_conf_attr.timeout_attr_id,
    &(radar_conf_attr.timeout_attr_val),
    false  // Don't check for access control
    );
    esp_zb_zcl_set_attribute_val(
    HA_ESP_ENDPOINT,
    RADAR_CONFIG_CLUSTER_ID,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    radar_conf_attr.update_rate_attr_id,
    &(radar_conf_attr.update_rate_attr_val),
    false  // Don't check for access control
    );
    esp_zb_zcl_set_attribute_val(
    HA_ESP_ENDPOINT,
    RADAR_CONFIG_CLUSTER_ID,
    ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    radar_conf_attr.angle_attr_id,
    &(radar_conf_attr.angle_attr_val),
    false  // Don't check for access control
    );

}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,
        .nwk_cfg.zed_cfg = {
            .ed_timeout = ED_AGING_TIMEOUT,
            .keep_alive = ED_KEEP_ALIVE,
        }};
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_ep_list_t *esp_zb_sensor_ep = custom_radar_sensor_ep_create(&radar_cfg);
    esp_zb_device_register(esp_zb_sensor_ep);

    esp_zb_zcl_reporting_info_t reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,
        .u.send_info.max_interval = 0,
        .u.send_info.def_min_interval = 1,
        .u.send_info.def_max_interval = 0,
        .u.send_info.delta.u16 = 1,
        .attr_id = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_custom_cluster_handlers_t radar_config_handler = {
    .cluster_id = RADAR_CONFIG_CLUSTER_ID,
    .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
    .check_value_cb = zb_zcl_radar_config_cluster_check_value_handler,
    .write_attr_cb = zb_zcl_radar_config_cluster_write_attr_handler,
    };
    esp_zb_zcl_custom_cluster_handlers_update(radar_config_handler);
    esp_zb_zcl_update_reporting_info(&reporting_info);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------GLOBAL FUNCTIONS-------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------------------------------------------------------------*/


void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            // ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in%s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new())
            {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            }
            else
            {
                ESP_LOGI(TAG, "Device rebooted");
                zb_handler_startup_attr_report();                
            }
        }
        else
        {
            ESP_LOGW(TAG, "%s failed with status: %s, retrying", esp_zb_zdo_signal_to_string(sig_type),
                     esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_INITIALIZATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK)
        {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zb_handler_startup_attr_report();
                    
        }
        else
        {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

void zigbeeHandler_Init(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = {
            .radio_mode = ZB_RADIO_MODE_NATIVE,
        },
        .host_config = {
            .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,
        },
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}

void zigbeeHandler_radarUpdate(uint8_t occupancy)
{
    if (esp_zb_bdb_dev_joined())
    {
        /* Update temperature sensor measured value */
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(HA_ESP_ENDPOINT,
                                     ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy, false);
        esp_zb_lock_release();
        
    }else 
    {
        ESP_LOGI(TAG, "Device is NOT connected to network");
    }
    ESP_LOGI(TAG, "Attributes range: %u, timeout: %u, update_rate: %u, angle: %u", radar_conf_attr.range_attr_val, radar_conf_attr.timeout_attr_val, radar_conf_attr.update_rate_attr_val, radar_conf_attr.angle_attr_val);
}
