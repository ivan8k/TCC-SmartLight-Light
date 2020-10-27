#include <stdio.h>

#include <driver/gpio.h>
#include <driver/adc.h>
#include <driver/rtc_io.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
//#include <driver/rtc_cntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_netif.h>
#include <esp_intr_alloc.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <soc/sens_reg.h>
#include <soc/rtc_periph.h>
#include <soc/rtc.h>
#include <esp32/ulp.h>
#include <ulp_main.h>

#include <lwip/err.h>
#include <lwip/sys.h>

#include <sdkconfig.h>

#include "protocol.c"

//#define HOST_IP "54.157.172.217"
#define HOST_IP "192.168.15.19"
//#define PORT 1026
#define PORT 40404

#define SSID      CONFIG_ESP_WIFI_SSID
#define PASS      CONFIG_ESP_WIFI_PASS
#define MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static int s_retry_num = 0;

static const char* TAG = "TEST";

static const proto8_t self_tag[] = {2};
static const proto8_t server_tag[] = {1};

#define SELF_TAG_SIZE sizeof(self_tag)
#define SERVER_TAG_SIZE sizeof(server_tag)

#define PIR_SEND_SIZE 3
#define LSENSOR_SEND_SIZE 1
#define PIR_MAX_SIZE 6
#define LSENSOR_MAX_SIZE 2

#define PIR_PDU_SIZE(X) (PIR_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2+PIR_PAYLOAD_SIZE(X))
#define LSENSOR_PDU_SIZE(X) (LSENSOR_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2+LSENSOR_PAYLOAD_SIZE(X))
#define OK_PDU_SIZE() (OK_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2)
#define KEEPALIVE_PDU_SIZE() (KEEPALIVE_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2)

#define I2C_SDA_GPIO_NUM 18
#define I2C_SCL_GPIO_NUM 19
#define I2C_CLK_SPEED 100000
#define I2C_PORT I2C_NUM_0
#define I2C_ACK_CHECK_EN 0x1
#define I2C_ACK_CHECK_DIS 0x0
#define I2C_ACK 0x0
#define I2C_NACK 0x1

#define BH1750_ADDR 0x23
#define BH1750_ONE_HRES 0x20
#define BH1750_ONE_HRES2 0x21
#define BH1750_CONT_HRES 0x10
#define BH1750_CONT_HRES2 0x11
#define BH1750_POWERDOWN 0x0
#define BH1750_POWERON 0x1

#define LIGHT_TRIGGER_SENSITIVITY 100
#define LIGHT_TRIGGER_DELAY 2

#define FALSE 0
#define TRUE 1

uint16_t light_data[2];
uint8_t light_index = 0;
uint16_t light_trigger = -1;
time_t light_trigger_time = -LIGHT_TRIGGER_DELAY;

uint8_t flag_send_light_trigger = FALSE;
uint8_t flag_active = TRUE;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
            ESP_LOGI(TAG, "Connecting...");
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            if (s_retry_num >= MAX_RETRY)
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGI(TAG, "Max retry reached.");
                return;
            }
            else
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retrying...");
            }
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        //ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected");
        //connected
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = 
    {
        .sta = 
        {
            .ssid = SSID,
            .password = PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Conectado a: %s", SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Erro na connexão wifi");
    }
    else
    {
        ESP_LOGW(TAG, "Evento desconhecido");
    }
    
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_got_ip));
    vEventGroupDelete(s_wifi_event_group);
}

int udp_comunicate(char payload[], int length, char response_buffer[], int response_length)//void* pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr;
    addr.sin_addr.s_addr = inet_addr(HOST_IP);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return -1;
    }

    int err = sendto(sock, payload, length, 0, (struct sockaddr*) &addr, sizeof(struct sockaddr_in));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Send error: %d", errno);
        return -1;
    }
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(struct sockaddr_in);
    int len = recvfrom(sock, response_buffer, response_length-1, 0, (struct sockaddr*) &source_addr, &socklen);
    if (len < 0)
    {
        ESP_LOGE(TAG, "Recv error: %d", errno);
        return len;
    }
    if (sock != -1)
    {
        ESP_LOGI(TAG, "Transmission Concluded.");
        shutdown(sock, 0);
        close(sock);
    }
    return len;
}

void reset_array(unsigned int array[], unsigned int size)
{
    for (unsigned int i = 0; i < size; i++)
    {
        array[i] = 0;
    }
       
}

void data16_to_proto8_t(uint16_t origin[], proto16_t size, proto8_t output_buffer[])
{
    for (int i = 0; i < size; i++)
    {
        output_buffer[i << 1] = (origin[i] >> 8) & UCHAR_MAX;
        output_buffer[(i << 1) + 1] = origin[i] & UCHAR_MAX;
    }
}

void send_ok();
uint8_t handle_response(char response[], int length)
{
    proto_data_unit_t data_unit = unmake_data_unit((uint8_t*) response);
    if (compare_header(head_ok, data_unit.header))
    {
        return TRUE;
    }
    else if (compare_header(head_idle, data_unit.header))
    {
        flag_active = FALSE;
        send_ok();
    }
    else if (compare_header(head_wake, data_unit.header))
    {
        flag_active = TRUE;
        send_ok();
    }
    else
    {
        return FALSE;
    }
    return TRUE;
    
}

void send_light_sensor()
{
    proto8_t payload[LSENSOR_PAYLOAD_SIZE(LSENSOR_MAX_SIZE)];
    proto16_t length = light_index+1;
    data16_to_proto8_t(light_data, length, payload);
    proto8_t data_unit[LSENSOR_PDU_SIZE(LSENSOR_MAX_SIZE)];
    int tcp_length = make_send_lsensor_data_unit(data_unit, (proto8_t*) self_tag, (proto8_t*) server_tag, payload, LSENSOR_PAYLOAD_SIZE(length));
    
    char* tcp_payload = (char*) data_unit;
    char response_buffer[256];
    int response_length = udp_comunicate(tcp_payload, tcp_length, response_buffer, 256);
    if (handle_response(response_buffer, response_length))
    {

    }
}
/*
void send_light_trigger()
{
    proto8_t payload[LSENSOR_PAYLOAD_SIZE(1)];
    proto16_t length = 1;
    data16_to_proto8_t(&(light_data[light_index]), 1, payload);
    proto8_t data_unit[LSENSOR_PDU_SIZE(1)];
    int tcp_length = make_send_ltrig_data_unit(data_unit, (proto8_t*) self_tag, (proto8_t*) server_tag, payload, LSENSOR_PAYLOAD_SIZE(1));
    
    char* tcp_payload = (char*) data_unit;
    char response_buffer[256];
    int response_length = udp_comunicate(tcp_payload, tcp_length, response_buffer, 256);
}*/

void send_keepalive()
{
    proto8_t data_unit[KEEPALIVE_PDU_SIZE()];
    int tcp_length = make_keepalive_data_unit(data_unit, (proto8_t*) self_tag, (proto8_t*) server_tag);
    
    char* tcp_payload = (char*) data_unit;
    char response_buffer[256];
    int response_length = udp_comunicate(tcp_payload, tcp_length, response_buffer, 256);
    if (handle_response(response_buffer, response_length))
    {

    }
}

void send_ok()
{
    proto8_t data_unit[OK_PDU_SIZE()];
    int tcp_length = make_ok_data_unit(data_unit, (proto8_t*) self_tag, (proto8_t*) server_tag);
    
    char* tcp_payload = (char*) data_unit;
    char response_buffer[256];
    int response_length = udp_comunicate(tcp_payload, tcp_length, response_buffer, 256);
}

void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_GPIO_NUM;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_GPIO_NUM;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_CLK_SPEED;
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

esp_err_t setup_i2c_light_sensor()
{
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, BH1750_CONT_HRES, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    /*
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_PTR_THRESHL, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_TL, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_TL, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);*/

    return err;
}

esp_err_t read_i2c_bh1750(uint16_t* data)
{
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BH1750_ADDR << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    uint8_t datah = 0xFF, datal=0x0;
    i2c_master_read_byte(cmd, &datah, I2C_ACK);
    i2c_master_read_byte(cmd, &datal, I2C_ACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    *data = (datah << 8) + datal;

    return err;
}
/*
void task_send_light_trigger(void* param)
{
    while (TRUE)
    {
        if (flag_send_light_trigger)
        {
            flag_send_light_trigger = 0;
            send_light_sensor();
            light_index = 0;
        }
        
    }
    
}*/

void app_main()
{
    i2c_master_init();
    setup_i2c_light_sensor();
    uint16_t test;
    time_t ref_time = 0;
    time_t now;
    for (;;)
    {
        read_i2c_bh1750(&light_data[light_index]);
        time(&now);
        if (abs((int16_t) (light_data[light_index] - light_trigger)) > LIGHT_TRIGGER_SENSITIVITY && light_trigger_time - now)
        {
            flag_send_light_trigger = TRUE;
            light_trigger_time = now;
        }
        if (now - ref_time >= 180)
        {
            send_light_sensor();
            if (++light_index == 2)
                light_index = 0;
            ref_time = now;
        }
        vTaskDelay(13);
        //printf("Time: %hu\n", test);
    }/*
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ulp_main_cpu_awake = 1;

    {
        ESP_LOGI(TAG, "Waking up, trying to send package.");
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_init());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_sta();
        wake_up_routine();
        esp_wifi_stop();
    }
    ESP_LOGI(TAG, "Entering deep sleep.");
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    //ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(60000000));

    ulp_main_cpu_awake = 0;
    esp_deep_sleep_start();*/
}
