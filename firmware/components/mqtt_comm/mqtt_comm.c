#include "mqtt_comm.h"
#include <stdbool.h>

static const char *TAG = "mqtt_example";
// Global MQTT client handle definition (declared extern in header)
esp_mqtt_client_handle_t client = NULL;
static int s_mqtt_connected = 0;



#define MAX_SUBSCRIBE_TOPICS 10
static mqtt_subscribe_item_t subscriptions[MAX_SUBSCRIBE_TOPICS];
static int subscription_count = 0;


static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        s_mqtt_connected = 1;
        /*msg_id = esp_mqtt_client_publish(client, "/topic/qos1", "data_3", 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);*/

        msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        // Re-subscribe any topics queued before connection
        for (int i = 0; i < subscription_count; ++i) {
            esp_mqtt_client_subscribe(client, subscriptions[i].topic, subscriptions[i].qos);
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
    s_mqtt_connected = 0;
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        // Prepare null-terminated copies (the raw pointers in event may NOT be null-terminated; lengths are provided).
        // Buffers sized defensively; truncate if topic/payload exceed buffer size - 1.
        char topic_nt[128];
        char data_nt[256];
        size_t tlen = (event->topic_len < sizeof(topic_nt)-1) ? event->topic_len : sizeof(topic_nt)-1;  // effective topic length copied
        size_t dlen = (event->data_len < sizeof(data_nt)-1) ? event->data_len : sizeof(data_nt)-1;      // effective payload length copied
        memcpy(topic_nt, event->topic, tlen); topic_nt[tlen] = '\0';   // copy and terminate topic
        memcpy(data_nt, event->data, dlen); data_nt[dlen] = '\0';      // copy and terminate payload
        ESP_LOGI(TAG, "TOPIC=%s", topic_nt);
        ESP_LOGI(TAG, "DATA=%s", data_nt);
        // Iterate all registered subscription entries and invoke the callback for exact matches.
        for (int i = 0; i < subscription_count; i++) {
            const char *sub_topic = subscriptions[i].topic;            // stored subscription topic string
            size_t sub_len = strlen(sub_topic);                        // its length (null-terminated)
            // Match only if lengths equal and contents identical (prevents partial prefix matches).
            if (sub_len == tlen && strncmp(topic_nt, sub_topic, tlen) == 0) {
                // Call the user-provided callback with safe, null-terminated topic & payload.
                subscriptions[i].callback(topic_nt, data_nt);
            }
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
#if CONFIG_BROKER_URL_FROM_STDIN
    char line[128];

    if (strcmp(mqtt_cfg.broker.address.uri, "FROM_STDIN") == 0) {
        int count = 0;
        printf("Please enter url of mqtt broker\n");
        while (count < 128) {
            int c = fgetc(stdin);
            if (c == '\n') {
                line[count] = '\0';
                break;
            } else if (c > 0 && c < 127) {
                line[count] = c;
                ++count;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        mqtt_cfg.broker.address.uri = line;
        printf("Broker url: %s\n", line);
    } else {
        ESP_LOGE(TAG, "Configuration mismatch: wrong broker url");
        abort();
    }
#endif /* CONFIG_BROKER_URL_FROM_STDIN */

    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

esp_err_t mqtt_publish(const char *topic, const char *payload, int qos) {
    if (!client) return ESP_ERR_INVALID_STATE;
    return esp_mqtt_client_publish(client, topic, payload, 0, qos, 0);
}

esp_err_t mqtt_subscribe_topic(const char *topic, int qos, void (*callback)(const char *, const char *)) {
    ESP_LOGI(TAG, "Subscribing to topic: %s with QoS %d", topic, qos);
    if (subscription_count >= MAX_SUBSCRIBE_TOPICS) return ESP_ERR_NO_MEM;
    subscriptions[subscription_count].topic = strdup(topic); // store a copy
    subscriptions[subscription_count].qos = qos;
    subscriptions[subscription_count].callback = callback;
    ++subscription_count;
    if (client) {
        return esp_mqtt_client_subscribe(client, topic, qos);
    }
    return ESP_OK;
}


esp_err_t mqtt_unsubscribe_topic(const char *topic) {
    if (!client) return ESP_ERR_INVALID_STATE;
    return esp_mqtt_client_unsubscribe(client, topic);
}

void mqtt_start(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("mqtt_client", ESP_LOG_VERBOSE);
    esp_log_level_set("mqtt_example", ESP_LOG_VERBOSE);
    esp_log_level_set("transport_base", ESP_LOG_VERBOSE);
    esp_log_level_set("esp-tls", ESP_LOG_VERBOSE);
    esp_log_level_set("transport", ESP_LOG_VERBOSE);
    esp_log_level_set("outbox", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    mqtt_app_start();
}

bool mqtt_is_connected(void)
{
    return s_mqtt_connected && client != NULL;
}

esp_err_t mqtt_wait_connected(int timeout_ms)
{
    const int interval_ms = 50;
    int waited = 0;
    while (!mqtt_is_connected()) {
        if (timeout_ms >= 0 && waited >= timeout_ms) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
        waited += interval_ms;
    }
    return ESP_OK;
}