#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include "i2c/i2c.h"
#include "bmp280/bmp280.h"

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

// this must be ahead of any mbedtls header files so the local mbedtls/config.h can be properly referenced
#include "ssl_connection.h"

#define MQTT_PUB_TOPIC ("esp8266/temp")
#define MQTT_SUB_TOPIC ("esp8266/control")
#define MQTT_PORT      (8883)
#define MQTT_MSG_LEN   (140) // ACTUALLY WAS THE PROBLEM ??????

#define PCF_ADDRESS	    0x38
#define MPU_ADDRESS	    0x68
#define BUS_I2C		    0
#define SCL             14
#define SDA             12
#define SENSOR_PERIOD   5000 // ms
#define GPIO_LED        2

/* certs, key, and endpoint */
extern char *ca_cert, *client_endpoint, *client_cert, *client_key;

static int wifi_alive = 0;
static int ssl_reset;
static SSLConnection *ssl_conn;
static QueueHandle_t publish_queue;
//////////////////////////////////////////////////////////////////////////////
typedef enum 
{
	BMP280_TEMPERATURE,
    BMP280_PRESSURE,
    BMP280_HUMIDITY
} bmp280_quantity;

bmp280_t bmp280_dev;

float convert_to_mbar(float pressure_in_pa)
{
    return pressure_in_pa / 100;
}

float read_bmp280 (bmp280_quantity selected_quantity)
{
    float temperature, pressure; //, humidity;

    // Force a measurement.
    bmp280_force_measurement(&bmp280_dev);

    // Wait for the measurement to complete.
    while (bmp280_is_measuring(&bmp280_dev))
    {};

    // Read measurement.
    // Go to ~/esp-open-rtos/extras/bmp280/bmp280.h -> this is where bmp280_read_float is defined
    // Humidity doesn't work, i.e. MQTT cannot publish that message (as far as I tried)
    bmp280_read_float(&bmp280_dev, &temperature, &pressure, NULL); //, &humidity);

    if (selected_quantity == BMP280_TEMPERATURE)
        return temperature;

    else if (selected_quantity == BMP280_PRESSURE)
        return convert_to_mbar(pressure);

    // else if (selected_quantity == BMP280_HUMIDITY)
    //     return humidity;

    return 0;
}

void read_temp_task(void* params)
{
    char msg[MQTT_MSG_LEN];
    float temp, pressure; //, humidity;
    int counter = 0;
    float tempAvg = 0, pressureAvg = 0;

    while(1)
    {
        // Measure temperature.
        temp = read_bmp280(BMP280_TEMPERATURE);
        // Measure pressure.
        pressure = read_bmp280(BMP280_PRESSURE);
        // Measure humidity.
        // humidity = read_bmp280(BMP280_HUMIDITY);

        // Average
        if (counter == 0) 
        {
            tempAvg = temp;
            pressureAvg = pressure;
        }
        else
        {
            tempAvg = ((tempAvg * counter) + temp) / (counter + 1);
            pressureAvg = ((pressureAvg * counter) + pressure) / (counter + 1);
        }

        // Store values in msg (mqtt payload in json format).
        snprintf(msg, MQTT_MSG_LEN, "{'T': %.2f C, 'P': %.2f mbar, 'Avg T': %.2f C, 'Avg P': %.2f mbar}", temp, pressure, tempAvg, pressureAvg);
        // printf("%d\n", strlen(msg));
        printf("%s\n", msg);
        counter += 1;

        // Attempt to push msg to queue.
        if (xQueueSend(publish_queue, (void *) msg, 0) == pdFALSE)
            printf("Publish queue overflow.\r\n");

        // Wait for a SENSOR_PERIOD.
        vTaskDelay(pdMS_TO_TICKS(SENSOR_PERIOD));
    }
}
//////////////////////////////////////////////////////////////////////////////
void low_pass_filter(float val)
{
    // already exists!
}
//////////////////////////////////////////////////////////////////////////////
void control_led(mqtt_message_t * message)
{
    if (!strncmp(message->payload, "ON", 2)) {
        printf("Turning on LED\r\n");
        gpio_write(GPIO_LED, 0);
    } else if (!strncmp(message->payload, "OFF", 3)) {
        printf("Turning off LED\r\n");
        gpio_write(GPIO_LED, 1);
    }
}
//////////////////////////////////////////////////////////////////////////////
static void topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;

    printf("Received: ");
    for (i = 0; i < md->topic->lenstring.len; ++i)
        printf("%c", md->topic->lenstring.data[i]);

    printf(" = ");
    for (i = 0; i < (int) message->payloadlen; ++i)
        printf("%c", ((char *) (message->payload))[i]);
    printf("\r\n");

    control_led(message);
}

static const char *get_my_id(void) 
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *) my_id))
        return NULL;
    for (i = 5; i >= 0; --i) {
        x = my_id[i] & 0x0F;
        if (x > 9)
            x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9)
            x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static int mqtt_ssl_read(mqtt_network_t * n, unsigned char* buffer, int len,
        int timeout_ms) 
{
    int r = ssl_read(ssl_conn, buffer, len, timeout_ms);
    if (r <= 0
            && (r != MBEDTLS_ERR_SSL_WANT_READ
                    && r != MBEDTLS_ERR_SSL_WANT_WRITE
                    && r != MBEDTLS_ERR_SSL_TIMEOUT)) {
        printf("%s: TLS read error (%d), resetting\n\r", __func__, r);
        ssl_reset = 1;
    };
    return r;
}

static int mqtt_ssl_write(mqtt_network_t* n, unsigned char* buffer, int len,
        int timeout_ms) 
{
    int r = ssl_write(ssl_conn, buffer, len, timeout_ms);
    if (r <= 0
            && (r != MBEDTLS_ERR_SSL_WANT_READ
                    && r != MBEDTLS_ERR_SSL_WANT_WRITE)) {
        printf("%s: TLS write error (%d), resetting\n\r", __func__, r);
        ssl_reset = 1;
    }
    return r;
}
//////////////////////////////////////////////////////////////////////////////
static void mqtt_task(void *pvParameters) {
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    ssl_conn = (SSLConnection *) malloc(sizeof(SSLConnection));
    while (1) 
    {
        if (!wifi_alive) 
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        printf("%s: started\n\r", __func__);
        ssl_reset = 0;
        ssl_init(ssl_conn);
        ssl_conn->ca_cert_str = ca_cert;
        ssl_conn->client_cert_str = client_cert;
        ssl_conn->client_key_str = client_key;

        mqtt_network_new(&network);
        network.mqttread = mqtt_ssl_read;
        network.mqttwrite = mqtt_ssl_write;

        printf("%s: connecting to MQTT server %s ... ", __func__, client_endpoint);
        ret = ssl_connect(ssl_conn, client_endpoint, MQTT_PORT);
        if (ret)
        {
            printf(":(\n");
            printf("error: %d\n\r", ret);
            ssl_destroy(ssl_conn);
            continue;
        }

        // Creat  client
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);
        data.willFlag          = 0;
        data.MQTTVersion       = 4;
        data.cleansession      = 1;
        data.clientID.cstring  = mqtt_client_id;
        data.username.cstring  = NULL;
        data.password.cstring  = NULL;
        data.keepAliveInterval = 1000;

        // Register client
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if (ret)
        {
            printf("error: %d\n\r", ret);
            ssl_destroy(ssl_conn);
            continue;
        }

        printf("done\r\n");
        mqtt_subscribe(&client, MQTT_SUB_TOPIC, MQTT_QOS1, topic_received);
        xQueueReset(publish_queue);

        while (wifi_alive && !ssl_reset)
        {
            // Add null terminator to message.
            char msg[MQTT_MSG_LEN - 1] = "\0";

            while (xQueueReceive(publish_queue, (void *) msg, 0) == pdTRUE)
            {
                /*
                    TickType_t task_tick = xTaskGetTickCount();
                    uint32_t   free_heap = xPortGetFreeHeapSize();
                    uint32_t   free_stack = uxTaskGetStackHighWaterMark(NULL);
                */
                // Assemble message.                
                mqtt_message_t  message;
                message.payload     = msg;
                message.payloadlen  = strlen(msg);
                message.dup         = 0;
                message.qos         = MQTT_QOS1;
                message.retained    = 0;

                // Publish message to the topic.
                ret = mqtt_publish(&client, MQTT_PUB_TOPIC, &message);
                if (ret != MQTT_SUCCESS)
                {
                    printf("error while publishing message: %d\n", ret);
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }

        printf("Connection dropped, request restart\n\r");
        ssl_destroy(ssl_conn);
    }
}

static void wifi_task(void *pvParameters) 
{
    uint8_t status = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = { .ssid = WIFI_SSID, .password =WIFI_PASS, };

    printf("%s: Connecting to WiFi\n\r", __func__);
    sdk_wifi_set_opmode (STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while (1) 
    {
        wifi_alive = 0;
        while ((status != STATION_GOT_IP) && (retries))
        {
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status);
            if (status == STATION_WRONG_PASSWORD)
            {
                printf("WiFi: wrong password\n\r");
                break;
            } 
            else if (status == STATION_NO_AP_FOUND)
            {
                printf("WiFi: AP not found\n\r");
                break;
            }
            else if (status == STATION_CONNECT_FAIL)
            {
                printf("WiFi: connection failed\r\n");
                break;
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
            --retries;
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP)
        {
            if (wifi_alive == 0)
            {
                printf("WiFi: Connected\n\r");
                wifi_alive = 1;
            }
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }

        wifi_alive = 0;
        printf("WiFi: disconnected\n\r");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
///////////////////////////////////////////////////////////////////////////

void user_init(void) 
{
    uart_set_baud(0, 115200);
    i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);

    // Init LED.
    gpio_enable(GPIO_LED, GPIO_OUTPUT);
    gpio_write(GPIO_LED, 1);

    // BMP280 configuration
	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_FORCED;
	bmp280_dev.i2c_dev.bus = BUS_I2C;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280_dev, &params);

    publish_queue = xQueueCreate(3, MQTT_MSG_LEN);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    xTaskCreate(&wifi_task, 
                "wifi_task",
                256,
                NULL,
                2,
                NULL);

    xTaskCreate(&read_temp_task,
                "read_temp_task",
                1000,
                NULL,
                2,
                NULL);

    xTaskCreate(&mqtt_task,
                "mqtt_task",
                2048,
                NULL,
                2,
                NULL);
}

