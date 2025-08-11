#include "config_manager.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>

static const char *TAG = "CONFIG_MANAGER";
static sphere_config_t current_config;
static bool config_initialized = false;

esp_err_t config_manager_init(void) {
    ESP_LOGI(TAG, "Initializing SPIFFS...");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "SPIFFS: %d KB total, %d KB used", total / 1024, used / 1024);
    
    // 設定ファイルが存在しない場合はデフォルト設定で作成
    if (!config_file_exists()) {
        ESP_LOGI(TAG, "Config file not found, creating with defaults");
        config_load_defaults(&current_config);
        ret = config_save_to_file(&current_config);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create default config file");
            return ret;
        }
    } else {
        ESP_LOGI(TAG, "Loading existing config file");
        ret = config_load_from_file(&current_config);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to load config, using defaults");
            config_load_defaults(&current_config);
        }
    }
    
    config_initialized = true;
    config_print_current();
    return ESP_OK;
}

esp_err_t config_manager_deinit(void) {
    esp_err_t ret = esp_vfs_spiffs_unregister(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to unregister SPIFFS (%s)", esp_err_to_name(ret));
    }
    config_initialized = false;
    return ret;
}

bool config_file_exists(void) {
    struct stat st;
    return stat(CONFIG_FILE_PATH, &st) == 0;
}

esp_err_t config_load_defaults(sphere_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    // 画像設定
    config->image.width = DEFAULT_IMAGE_WIDTH;
    config->image.height = DEFAULT_IMAGE_HEIGHT;
    config->image.fps = DEFAULT_IMAGE_FPS;
    config->image.quality = DEFAULT_IMAGE_QUALITY;
    strncpy(config->image.format, DEFAULT_IMAGE_FORMAT, sizeof(config->image.format) - 1);
    
    // LED設定
    config->led.count = DEFAULT_LED_COUNT;
    config->led.brightness = DEFAULT_LED_BRIGHTNESS;
    config->led.strip_count = DEFAULT_LED_STRIP_COUNT;
    config->led.leds_per_strip = DEFAULT_LEDS_PER_STRIP;
    config->led.update_rate_hz = DEFAULT_LED_UPDATE_RATE;
    
    // IMU設定
    config->imu.sample_rate_hz = DEFAULT_IMU_SAMPLE_RATE;
    config->imu.calibration_enabled = true;
    config->imu.orientation_offset[0] = 1.0f; // w
    config->imu.orientation_offset[1] = 0.0f; // x
    config->imu.orientation_offset[2] = 0.0f; // y
    config->imu.orientation_offset[3] = 0.0f; // z
    config->imu.filter_strength = DEFAULT_IMU_FILTER_STRENGTH;
    
    // 通信設定
    memset(config->communication.wifi_ssid, 0, sizeof(config->communication.wifi_ssid));
    memset(config->communication.wifi_password, 0, sizeof(config->communication.wifi_password));
    config->communication.ros2_domain_id = DEFAULT_ROS2_DOMAIN_ID;
    strncpy(config->communication.ros2_namespace, "sphere", sizeof(config->communication.ros2_namespace) - 1);
    config->communication.update_rate_hz = DEFAULT_COMM_UPDATE_RATE;
    
    // システム設定
    config->system.log_level = DEFAULT_LOG_LEVEL;
    config->system.debug_mode = false;
    config->system.watchdog_timeout_ms = DEFAULT_WATCHDOG_TIMEOUT;
    config->system.performance_monitoring = true;
    
    ESP_LOGI(TAG, "Loaded default configuration");
    return ESP_OK;
}

esp_err_t config_load_from_file(sphere_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    FILE *file = fopen(CONFIG_FILE_PATH, "r");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open config file");
        return ESP_ERR_NOT_FOUND;
    }
    
    // ファイルサイズ取得
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (file_size <= 0 || file_size > 8192) {
        ESP_LOGE(TAG, "Invalid config file size: %ld", file_size);
        fclose(file);
        return ESP_ERR_INVALID_SIZE;
    }
    
    char *buffer = malloc(file_size + 1);
    if (!buffer) {
        ESP_LOGE(TAG, "Failed to allocate memory for config file");
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    
    size_t read_size = fread(buffer, 1, file_size, file);
    buffer[read_size] = '\0';
    fclose(file);
    
    cJSON *json = cJSON_Parse(buffer);
    free(buffer);
    
    if (!json) {
        ESP_LOGE(TAG, "Failed to parse JSON config");
        return ESP_ERR_INVALID_ARG;
    }
    
    // JSON から設定を読み込み
    cJSON *image = cJSON_GetObjectItem(json, "image");
    if (image) {
        config->image.width = cJSON_GetObjectItem(image, "width")->valueint;
        config->image.height = cJSON_GetObjectItem(image, "height")->valueint;
        config->image.fps = cJSON_GetObjectItem(image, "fps")->valueint;
        config->image.quality = cJSON_GetObjectItem(image, "quality")->valueint;
        strncpy(config->image.format, cJSON_GetObjectItem(image, "format")->valuestring, 
                sizeof(config->image.format) - 1);
    }
    
    cJSON *led = cJSON_GetObjectItem(json, "led");
    if (led) {
        config->led.count = cJSON_GetObjectItem(led, "count")->valueint;
        config->led.brightness = cJSON_GetObjectItem(led, "brightness")->valueint;
        config->led.strip_count = cJSON_GetObjectItem(led, "strip_count")->valueint;
        config->led.leds_per_strip = cJSON_GetObjectItem(led, "leds_per_strip")->valueint;
        config->led.update_rate_hz = cJSON_GetObjectItem(led, "update_rate_hz")->valueint;
    }
    
    cJSON *imu = cJSON_GetObjectItem(json, "imu");
    if (imu) {
        config->imu.sample_rate_hz = cJSON_GetObjectItem(imu, "sample_rate_hz")->valueint;
        config->imu.calibration_enabled = cJSON_IsTrue(cJSON_GetObjectItem(imu, "calibration_enabled"));
        
        cJSON *offset = cJSON_GetObjectItem(imu, "orientation_offset");
        if (cJSON_IsArray(offset) && cJSON_GetArraySize(offset) == 4) {
            for (int i = 0; i < 4; i++) {
                config->imu.orientation_offset[i] = cJSON_GetArrayItem(offset, i)->valuedouble;
            }
        }
        config->imu.filter_strength = cJSON_GetObjectItem(imu, "filter_strength")->valueint;
    }
    
    cJSON *comm = cJSON_GetObjectItem(json, "communication");
    if (comm) {
        strncpy(config->communication.wifi_ssid, 
                cJSON_GetObjectItem(comm, "wifi_ssid")->valuestring,
                sizeof(config->communication.wifi_ssid) - 1);
        strncpy(config->communication.wifi_password, 
                cJSON_GetObjectItem(comm, "wifi_password")->valuestring,
                sizeof(config->communication.wifi_password) - 1);
        config->communication.ros2_domain_id = cJSON_GetObjectItem(comm, "ros2_domain_id")->valueint;
        strncpy(config->communication.ros2_namespace,
                cJSON_GetObjectItem(comm, "ros2_namespace")->valuestring,
                sizeof(config->communication.ros2_namespace) - 1);
        config->communication.update_rate_hz = cJSON_GetObjectItem(comm, "update_rate_hz")->valueint;
    }
    
    cJSON *system = cJSON_GetObjectItem(json, "system");
    if (system) {
        config->system.log_level = cJSON_GetObjectItem(system, "log_level")->valueint;
        config->system.debug_mode = cJSON_IsTrue(cJSON_GetObjectItem(system, "debug_mode"));
        config->system.watchdog_timeout_ms = cJSON_GetObjectItem(system, "watchdog_timeout_ms")->valueint;
        config->system.performance_monitoring = cJSON_IsTrue(cJSON_GetObjectItem(system, "performance_monitoring"));
    }
    
    cJSON_Delete(json);
    ESP_LOGI(TAG, "Successfully loaded configuration from file");
    return ESP_OK;
}

esp_err_t config_save_to_file(const sphere_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    cJSON *json = cJSON_CreateObject();
    if (!json) {
        ESP_LOGE(TAG, "Failed to create JSON object");
        return ESP_ERR_NO_MEM;
    }
    
    // 画像設定
    cJSON *image = cJSON_CreateObject();
    cJSON_AddNumberToObject(image, "width", config->image.width);
    cJSON_AddNumberToObject(image, "height", config->image.height);
    cJSON_AddNumberToObject(image, "fps", config->image.fps);
    cJSON_AddNumberToObject(image, "quality", config->image.quality);
    cJSON_AddStringToObject(image, "format", config->image.format);
    cJSON_AddItemToObject(json, "image", image);
    
    // LED設定
    cJSON *led = cJSON_CreateObject();
    cJSON_AddNumberToObject(led, "count", config->led.count);
    cJSON_AddNumberToObject(led, "brightness", config->led.brightness);
    cJSON_AddNumberToObject(led, "strip_count", config->led.strip_count);
    cJSON_AddNumberToObject(led, "leds_per_strip", config->led.leds_per_strip);
    cJSON_AddNumberToObject(led, "update_rate_hz", config->led.update_rate_hz);
    cJSON_AddItemToObject(json, "led", led);
    
    // IMU設定
    cJSON *imu = cJSON_CreateObject();
    cJSON_AddNumberToObject(imu, "sample_rate_hz", config->imu.sample_rate_hz);
    cJSON_AddBoolToObject(imu, "calibration_enabled", config->imu.calibration_enabled);
    
    cJSON *offset = cJSON_CreateArray();
    for (int i = 0; i < 4; i++) {
        cJSON_AddItemToArray(offset, cJSON_CreateNumber(config->imu.orientation_offset[i]));
    }
    cJSON_AddItemToObject(imu, "orientation_offset", offset);
    cJSON_AddNumberToObject(imu, "filter_strength", config->imu.filter_strength);
    cJSON_AddItemToObject(json, "imu", imu);
    
    // 通信設定
    cJSON *comm = cJSON_CreateObject();
    cJSON_AddStringToObject(comm, "wifi_ssid", config->communication.wifi_ssid);
    cJSON_AddStringToObject(comm, "wifi_password", config->communication.wifi_password);
    cJSON_AddNumberToObject(comm, "ros2_domain_id", config->communication.ros2_domain_id);
    cJSON_AddStringToObject(comm, "ros2_namespace", config->communication.ros2_namespace);
    cJSON_AddNumberToObject(comm, "update_rate_hz", config->communication.update_rate_hz);
    cJSON_AddItemToObject(json, "communication", comm);
    
    // システム設定
    cJSON *system = cJSON_CreateObject();
    cJSON_AddNumberToObject(system, "log_level", config->system.log_level);
    cJSON_AddBoolToObject(system, "debug_mode", config->system.debug_mode);
    cJSON_AddNumberToObject(system, "watchdog_timeout_ms", config->system.watchdog_timeout_ms);
    cJSON_AddBoolToObject(system, "performance_monitoring", config->system.performance_monitoring);
    cJSON_AddItemToObject(json, "system", system);
    
    char *json_string = cJSON_Print(json);
    cJSON_Delete(json);
    
    if (!json_string) {
        ESP_LOGE(TAG, "Failed to serialize JSON");
        return ESP_ERR_NO_MEM;
    }
    
    FILE *file = fopen(CONFIG_FILE_PATH, "w");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open config file for writing");
        free(json_string);
        return ESP_FAIL;
    }
    
    size_t written = fwrite(json_string, 1, strlen(json_string), file);
    fclose(file);
    free(json_string);
    
    if (written <= 0) {
        ESP_LOGE(TAG, "Failed to write config file");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Configuration saved to file successfully");
    return ESP_OK;
}

const sphere_config_t* config_get_current(void) {
    return config_initialized ? &current_config : NULL;
}

void config_print_current(void) {
    if (!config_initialized) {
        ESP_LOGW(TAG, "Config not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Current Configuration:");
    ESP_LOGI(TAG, "  Image: %dx%d @ %d fps, quality=%d, format=%s", 
             current_config.image.width, current_config.image.height,
             current_config.image.fps, current_config.image.quality,
             current_config.image.format);
    ESP_LOGI(TAG, "  LED: count=%d, brightness=%d, strips=%d, rate=%d Hz",
             current_config.led.count, current_config.led.brightness,
             current_config.led.strip_count, current_config.led.update_rate_hz);
    ESP_LOGI(TAG, "  IMU: rate=%d Hz, calibration=%s, filter=%d",
             current_config.imu.sample_rate_hz,
             current_config.imu.calibration_enabled ? "enabled" : "disabled",
             current_config.imu.filter_strength);
    ESP_LOGI(TAG, "  System: log_level=%d, debug=%s",
             current_config.system.log_level,
             current_config.system.debug_mode ? "enabled" : "disabled");
}

esp_err_t config_validate(const sphere_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    
    // 画像設定の検証
    if (config->image.width < 64 || config->image.width > 1920) {
        ESP_LOGE(TAG, "Invalid image width: %d", config->image.width);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->image.height < 64 || config->image.height > 1080) {
        ESP_LOGE(TAG, "Invalid image height: %d", config->image.height);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->image.fps < 1 || config->image.fps > 60) {
        ESP_LOGE(TAG, "Invalid image fps: %d", config->image.fps);
        return ESP_ERR_INVALID_ARG;
    }
    
    // LED設定の検証
    if (config->led.count < 1 || config->led.count > 2000) {
        ESP_LOGE(TAG, "Invalid LED count: %d", config->led.count);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->led.strip_count < 1 || config->led.strip_count > 8) {
        ESP_LOGE(TAG, "Invalid strip count: %d", config->led.strip_count);
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Configuration validation passed");
    return ESP_OK;
}

esp_err_t config_backup(void) {
    FILE *src = fopen(CONFIG_FILE_PATH, "r");
    if (!src) {
        ESP_LOGE(TAG, "Failed to open source config file");
        return ESP_ERR_NOT_FOUND;
    }
    
    FILE *dst = fopen(CONFIG_BACKUP_PATH, "w");
    if (!dst) {
        ESP_LOGE(TAG, "Failed to open backup config file");
        fclose(src);
        return ESP_FAIL;
    }
    
    char buffer[512];
    size_t bytes;
    while ((bytes = fread(buffer, 1, sizeof(buffer), src)) > 0) {
        fwrite(buffer, 1, bytes, dst);
    }
    
    fclose(src);
    fclose(dst);
    
    ESP_LOGI(TAG, "Configuration backed up successfully");
    return ESP_OK;
}

// 個別設定更新関数の実装
esp_err_t config_update_image_settings(uint16_t width, uint16_t height, uint8_t fps, uint8_t quality) {
    if (!config_initialized) return ESP_ERR_INVALID_STATE;
    
    current_config.image.width = width;
    current_config.image.height = height;
    current_config.image.fps = fps;
    current_config.image.quality = quality;
    
    return config_save_to_file(&current_config);
}