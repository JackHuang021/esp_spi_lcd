#include "smartconfig.h"

static EventGroupHandle_t wifi_event_group = NULL;

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *tag = "smartconfig";
static bool smartconfig_run = false;

static esp_err_t save_wifi_credentials(const char *ssid, const char *password)
{
    esp_err_t ret = ESP_OK;
    nvs_handle_t nvs_handle;

    ret = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Error (%s) opening NVS handle!", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(nvs_handle, "ssid", ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "set ssid failed, return %s", esp_err_to_name(ret));
        goto out;
    }

    ret = nvs_set_str(nvs_handle, "password", password);
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "set password failed, return %s", esp_err_to_name(ret));
        goto out;
    }

    ret = nvs_commit(nvs_handle);

    if (ret == ESP_OK)
        ESP_LOGI(tag, "WiFi credentials saved: SSID=%s, PASSWORD=%s", ssid, password);
    else
        ESP_LOGE(tag, "save wifi info failed, return %s", esp_err_to_name(ret));
out:
    nvs_close(nvs_handle);
    return ret;
}

static bool read_wifi_credentials(char *ssid, char *password)
{
    esp_err_t ret = ESP_OK;
    size_t ssid_len = MAX_SSID_LEN;
    size_t pwd_len = MAX_PASSPHRASE_LEN;

    nvs_handle_t nvs_handle;
    ret = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGI(tag, "open nvs flash failed, return %s", esp_err_to_name(ret));
        return false;
    }

    if (nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len) != ESP_OK ||
        nvs_get_str(nvs_handle, "password", password, &pwd_len) != ESP_OK) {
        nvs_close(nvs_handle);
        ESP_LOGE(tag, "read  WiFi credentials failed");
        return false;
    }

    nvs_close(nvs_handle);
    ESP_LOGI(tag, "read  WiFi credentials: SSID=%s, PASSWORD=%s", ssid, password);

    return true;
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
        }
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }

    if (event_base ==  SC_EVENT) {
        switch (event_id) {
        case SC_EVENT_SCAN_DONE:
            ESP_LOGI(tag, "Scan done");
            break;
        case SC_EVENT_FOUND_CHANNEL:
            ESP_LOGI(tag, "Found channel");
            break;
        case SC_EVENT_GOT_SSID_PSWD:
            ESP_LOGI(tag, "got SSID and password");
            smartconfig_event_got_ssid_pswd_t *event = (smartconfig_event_got_ssid_pswd_t *)event_data;
            wifi_config_t wifi_config;
            char ssid[MAX_SSID_LEN] = {0};
            char password[MAX_PASSPHRASE_LEN] = {0};

            bzero(&wifi_config, sizeof(wifi_config));
            memcpy(wifi_config.sta.ssid, event->ssid, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, event->password, sizeof(wifi_config.sta.password));

            wifi_config.sta.bssid_set = event->bssid_set;
            if (wifi_config.sta.bssid_set) {
                ESP_LOGI(tag, "set mac address target of AP: "MACSTR"", MAC2STR(event->bssid));
                memcpy(wifi_config.sta.bssid, event->bssid, sizeof(wifi_config.sta.bssid));
            }

            memcpy(ssid, event->ssid, sizeof(event->ssid));
            memcpy(password, event->password, sizeof(event->password));
            save_wifi_credentials(ssid, password);
            ESP_LOGI(tag, "SSID: %s", ssid);
            ESP_LOGI(tag, "password: %s", password);
            esp_wifi_disconnect();
            esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
            esp_wifi_connect();
            break;
        case SC_EVENT_SEND_ACK_DONE:
            xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
            break;
        default:
            break;
        }

    }
}

static void smartconfig_task(void *param)
{
    EventBits_t bits;
    static bool startup = true;

    while (1) {
        if (startup) {
            ESP_LOGI(tag, "wait 10S to connect wifi");
            bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                                            pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));
            if (!(bits & CONNECTED_BIT))
                smartconfig_run = true;
            startup = false;
        }

        if (smartconfig_run) {
            smartconfig_start_config_t config = SMARTCONFIG_START_CONFIG_DEFAULT();
            esp_smartconfig_set_type(SC_TYPE_ESPTOUCH);
            esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
            esp_smartconfig_start(&config);
            // 等待wifi连接
            bits = xEventGroupWaitBits(wifi_event_group, 
                                        CONNECTED_BIT | ESPTOUCH_DONE_BIT,
                                        true, false, portMAX_DELAY);
            if(bits & CONNECTED_BIT) {
                ESP_LOGI(tag, "WiFi Connected to ap");
            }
            if(bits & ESPTOUCH_DONE_BIT) {
                ESP_LOGI(tag, "smartconfig over");
                esp_smartconfig_stop();
            }
            smartconfig_run = false;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void init_wifi(void)
{
    char ssid[MAX_SSID_LEN] = {0};
    char password[MAX_PASSPHRASE_LEN] = {0};
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    if (read_wifi_credentials(ssid, password)) {
        wifi_config_t config;
        bzero(&config, sizeof(config));
        memcpy(config.sta.ssid, ssid, sizeof(config.sta.ssid));
        memcpy(config.sta.password, password, sizeof(config.sta.password));
        esp_wifi_set_config(WIFI_IF_STA, &config);
        esp_wifi_connect();
    }

    xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
    ESP_LOGI(tag, "start smartconfig");
}