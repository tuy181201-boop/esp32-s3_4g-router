#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_netif_ppp.h>
#include <esp_netif_net_nat.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <esp_mac.h>
#include <esp_wifi.h>
#include <esp_https_server.h>
#include <mdns.h>
#include <mqtt_client.h>
#include <cJSON.h>
#include <sys/param.h>

// ============================================
// 1. Настройки PIN-ов и UART для модема A7670E
// ============================================
#define MODEM_UART_NUM     UART_NUM_1
#define MODEM_TX_PIN       GPIO_NUM_17
#define MODEM_RX_PIN       GPIO_NUM_18
#define MODEM_PWRKEY_PIN   GPIO_NUM_14
#define MODEM_DTR_PIN      GPIO_NUM_15   // опционально
#define MODEM_BAUD         115200

#define APN                ""
#define APN_USER           ""
#define APN_PASS           ""

// ============================================
// 2. Настройки Wi-Fi (Точка доступа)
// ============================================
#define AP_SSID            ""
#define AP_PASSWORD        ""
#define AP_CHANNEL         6
#define AP_MAX_CONN        4
#define AP_IP              "192.168.4.1"
#define AP_NETMASK         "255.255.255.0"
#define AP_GATEWAY         "192.168.4.1"

// ============================================
// 3. Настройки MQTT (телеметрия)
// ============================================
#define MQTT_BROKER_URI    "mqtt://broker.emqx.io"
#define MQTT_TOPIC_STATUS  "esp32/4g_router/status"
#define MQTT_TOPIC_CMD     "esp32/4g_router/cmd"

// ============================================
// 4. Семисегментный индикатор (74HC595)
// ============================================
#define HC595_DS   16
#define HC595_SHCP 17
#define HC595_STCP 18
#define COMMON_CATHODE true

const byte symbolPatterns[16][8] = {
  {1,1,1,1,1,1,0,0}, {0,1,1,0,0,0,0,0}, {1,1,0,1,1,0,1,0}, {1,1,1,1,0,0,1,0},
  {0,1,1,0,0,1,1,0}, {1,0,1,1,0,1,1,0}, {1,0,1,1,1,1,1,0}, {1,1,1,0,0,0,0,0},
  {1,1,1,1,1,1,1,0}, {1,1,1,1,0,1,1,0}, {1,1,1,0,1,1,1,0}, {0,0,1,1,1,1,1,0},
  {1,0,0,1,1,1,0,0}, {0,1,1,1,1,0,1,0}, {1,0,0,1,1,1,1,0}, {1,0,0,0,1,1,1,0}
};

static const char *TAG = "4G_ROUTER";
static esp_netif_t *ppp_netif = NULL;
static bool modem_connected = false;
static int last_signal = 0;
static char operator_name[32] = "Unknown";
static uint32_t last_publish = 0;
static esp_mqtt_client_handle_t mqtt_client = NULL;

QueueHandle_t clientsQueue;
QueueHandle_t otaProgressQueue;

// ============================================
// 5. Семисегментный индикатор (управление)
// ============================================
void initDisplay() {
    pinMode(HC595_DS, OUTPUT);
    pinMode(HC595_SHCP, OUTPUT);
    pinMode(HC595_STCP, OUTPUT);
    digitalWrite(HC595_STCP, LOW);
    digitalWrite(HC595_SHCP, LOW);
    digitalWrite(HC595_DS, LOW);
    shiftOut(HC595_DS, HC595_SHCP, LSBFIRST, 0);
    digitalWrite(HC595_STCP, HIGH);
}

void sendByteTo595(byte data) {
    digitalWrite(HC595_STCP, LOW);
    shiftOut(HC595_DS, HC595_SHCP, LSBFIRST, data);
    digitalWrite(HC595_STCP, HIGH);
}

void displaySymbol(uint8_t symbol) {
    if (symbol > 15) symbol = 15;
    byte segData = 0;
    for (int i = 0; i < 8; i++) {
        bool state = symbolPatterns[symbol][i];
        if (!COMMON_CATHODE) state = !state;
        if (state) segData |= (1 << i);
    }
    sendByteTo595(segData);
}

void TaskDisplay(void *pvParameters) {
    initDisplay();
    displaySymbol(0);
    int value;
    for (;;) {
        if (xQueueReceive(otaProgressQueue, &value, 0) == pdTRUE) {
            if (value == -1) {
                displaySymbol(0);
            } else {
                int symbol = map(value, 0, 100, 0, 15);
                if (symbol < 0) symbol = 0;
                if (symbol > 15) symbol = 15;
                displaySymbol(symbol);
            }
            continue;
        }
        if (xQueueReceive(clientsQueue, &value, pdMS_TO_TICKS(200)) == pdTRUE) {
            if (value < 0) value = 0;
            if (value > 15) value = 15;
            displaySymbol(value);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ============================================
// 6. AT-команды и инициализация модема
// ============================================
void modem_send_at_cmd(const char *cmd, uint32_t timeout_ms) {
    uart_write_bytes(MODEM_UART_NUM, cmd, strlen(cmd));
    uart_write_bytes(MODEM_UART_NUM, "\r\n", 2);
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
}

bool modem_init() {
    gpio_set_level(MODEM_PWRKEY_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(MODEM_PWRKEY_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));

    modem_send_at_cmd("ATE0", 500);
    modem_send_at_cmd("AT+CMEE=2", 500);
    modem_send_at_cmd("AT+CFUN=1", 5000);

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", APN);
    modem_send_at_cmd(cmd, 5000);

    modem_send_at_cmd("AT+CEREG?", 3000);
    return true;
}

// ============================================
// 7. Обработчики событий PPP и сети
// ============================================
void ppp_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case NETIF_PPP_CONNECTED:
            ESP_LOGI(TAG, "PPP соединение установлено");
            modem_connected = true;
            displaySymbol(2);
            break;
        case NETIF_PPP_GOT_IP:
            ESP_LOGI(TAG, "Получен IP от оператора");
            displaySymbol(3);
            esp_netif_t *ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
            if (ap_netif && ppp_netif) {
                esp_netif_nat_set_enabled(ap_netif, true);
                esp_netif_set_default_netif(ppp_netif);
                ESP_LOGI(TAG, "NAT включен, интернет раздается");
            }
            break;
        case NETIF_PPP_LOST_IP:
            ESP_LOGW(TAG, "Потерян IP");
            modem_connected = false;
            displaySymbol(0);
            break;
    }
}

// ============================================
// 8. RSSI и оператор (заглушки, доработать)
// ============================================
int get_modem_rssi() { return -67; }
const char* get_modem_operator() { return "Tele2"; }

// ============================================
// 9. MQTT
// ============================================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    if (event_id == MQTT_EVENT_DATA) {
        if (strncmp(event->topic, MQTT_TOPIC_CMD, event->topic_len) == 0) {
            if (strncmp(event->data, "reboot", event->data_len) == 0) {
                ESP_LOGI(TAG, "Перезагрузка по MQTT");
                esp_restart();
            }
        }
    }
}

void publish_telemetry() {
    if (!mqtt_client) return;
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "clients", wifi_softap_get_station_num());
    cJSON_AddNumberToObject(root, "rssi", get_modem_rssi());
    cJSON_AddStringToObject(root, "operator", get_modem_operator());
    cJSON_AddStringToObject(root, "status", modem_connected ? "connected" : "disconnected");
    cJSON_AddNumberToObject(root, "uptime", esp_timer_get_time() / 1000000);
    char *json_str = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_STATUS, json_str, 0, 0, 0);
    cJSON_Delete(root);
    free(json_str);
}

// ============================================
// 10. Веб-сервер (страница /status)
// ============================================
esp_err_t status_get_handler(httpd_req_t *req) {
    char resp[1024];
    snprintf(resp, sizeof(resp),
        "<html><body>"
        "<h1>Статус 4G роутера</h1>"
        "<p>Клиентов: %d</p>"
        "<p>RSSI: %d dBm</p>"
        "<p>Оператор: %s</p>"
        "<p>4G статус: %s</p>"
        "</body></html>",
        wifi_softap_get_station_num(),
        get_modem_rssi(),
        get_modem_operator(),
        modem_connected ? "Подключен" : "Нет подключения");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, strlen(resp));
    return ESP_OK;
}

void start_webserver() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_get_handler };
        httpd_register_uri_handler(server, &status_uri);
        ESP_LOGI(TAG, "HTTP сервер запущен");
    }
}

// ============================================
// 11. Основная функция
// ============================================
extern "C" void app_main() {
    ESP_LOGI(TAG, "Запуск 4G роутера на ESP32-S3 + A7670E");

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();

    uart_config_t uart_config = {
        .baud_rate = MODEM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(MODEM_UART_NUM, &uart_config);
    uart_set_pin(MODEM_UART_NUM, MODEM_TX_PIN, MODEM_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(MODEM_UART_NUM, 4096, 4096, 0, NULL, 0);
    gpio_set_direction(MODEM_PWRKEY_PIN, GPIO_MODE_OUTPUT);

    // Wi-Fi AP
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t ap_config = {
        .ap = {
            .ssid = AP_SSID,
            .password = AP_PASSWORD,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_WPA3_PSK
        },
    };
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();

    // PPP
    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_PPP();
    ppp_netif = esp_netif_new(&netif_config);
    esp_netif_ppp_set_params(ppp_netif, APN);
    esp_netif_ppp_set_auth(ppp_netif, APN_USER, APN_PASS);
    esp_netif_attach_ppp(ppp_netif, MODEM_UART_NUM);
    esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP, &ppp_event_handler, NULL);
    esp_event_handler_register(NETIF_PPP_EVENT, NETIF_PPP_CONNECTED, &ppp_event_handler, NULL);
    esp_netif_ppp_start(ppp_netif);

    modem_init();

    // Очереди и задача дисплея
    clientsQueue = xQueueCreate(1, sizeof(int));
    otaProgressQueue = xQueueCreate(1, sizeof(int));
    xTaskCreatePinnedToCore(TaskDisplay, "DisplayTask", 4096, NULL, 1, NULL, 0);

    // MQTT
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_CMD, 0);

    start_webserver();

    while (1) {
        if (modem_connected && (esp_timer_get_time() / 1000000 - last_publish >= 30)) {
            publish_telemetry();
            last_publish = esp_timer_get_time() / 1000000;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
