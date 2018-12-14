#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "soc/rtc_cntl_reg.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "config.h"
#include "debug.h"

#include "mqtt.h"
#include "os.h"
#include "onenet.h"

#include "driver/uart.h"
#include "soc/uart_struct.h"


static bool onenet_initialised = false;
static TaskHandle_t xOneNetTask = NULL;


#define UART2_TXD_PIN   (23)
#define UART2_RXD_PIN   (22)

#define UART0_TXD_PIN (GPIO_NUM_4)
#define UART0_RXD_PIN (GPIO_NUM_5)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define ECHO_TEST_RTS   (18)

// CTS is not used in RS485 Half-Duplex Mode
#define ECHO_TEST_CTS  UART_PIN_NO_CHANGE

#define BUF_SIZE        (127)
#define BAUD_RATE       (115200)

// Read packet timeout
#define PACKET_READ_TICS        (100 / portTICK_RATE_MS)
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
#define ECHO_UART0_PORT         (UART_NUM_0)
#define ECHO_UART2_PORT         (UART_NUM_2)

unsigned char moto_lock[] =     {0xFF,0xFF,0xFE,0x04,0x03,0x28,0x01,0xD1};
unsigned char moto_unlock[] =   {0xFF,0xFF,0xFE,0x04,0x03,0x28,0x00,0xD2};
unsigned char moto_move[] =     {0xFF,0xFF,0xFE,0x05,0x03,0x2a,0x08,0x00,0xC7};
unsigned char free_data[20];
unsigned char free_signal[8];

void sendDataMoto(int uart_num,unsigned char *data,unsigned char moveEnable,unsigned int Location,unsigned int len)
{
    unsigned char check,checkBit;
//    if(uart_num==uart0_num)
//    static const char *TAG = "UART0_TX_TASK";
//    else if(uart_num==uart2_num)
    static const char *TAG = "UART2_TX_TASK";

    check=0;
    if(moveEnable==1)
    {
        moto_move[6]=Location/256;
        moto_move[7]=Location%256;
        for (int i = 2; i < 8 ; i++)
        {
            check = check + moto_move[i];
        }
        checkBit = check % 256;
        checkBit = 255 - checkBit; 
        moto_move[8]=checkBit;

        ESP_LOGI(TAG, "Wrote %d bytes", 8);
        printf("[ ");
            for (int i = 0; i < 8; i++) {
                printf("0x%.2X ", (uint8_t)moto_lock[i]);
                uart_write_bytes(uart_num, (const char*)&moto_lock[i], 1);
                // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
                if (moto_lock[i] == '\r') {
                    vTaskDelay(20 / portTICK_PERIOD_MS);
                  //  uart_write_bytes(uart_num, "\n", 1);
                }
            }
            printf("] \n");
        //uart_write_bytes(uart_num, "]\r\n", 3);
    }  

    ESP_LOGI(TAG, "Wrote %d bytes", len);
    printf("[ ");
        for (int i = 0; i < len; i++) {
            printf("0x%.2X ", (uint8_t)data[i]);
            uart_write_bytes(uart_num, (const char*)&data[i], 1);
            // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
            if (data[i] == '\r') {
            //    uart_write_bytes(uart_num, "\n", 1);
            }
        }
        printf("] \n");
        free_signal[0]=1;
    //uart_write_bytes(uart_num, "]\r\n", 3);
    
}

void onenet_task(void *param)
{
   mqtt_client* client = (mqtt_client *)param;
   uint32_t val;

   while(1) {
        /**
         * Replace this *val* with real captured value by your sensor,
         * Here, we just use a rand number which ranges form 15 to 35. 
         */
        val = os_random() % 20 + 15;

        char buf[128];
        memset(buf, 0, sizeof(buf));
        sprintf(&buf[3], "{\"%s\":%d}", ONENET_DATA_STREAM, val);
        uint16_t len = strlen(&buf[3]);
        buf[0] = data_type_simple_json_without_time;
        buf[1] = len >> 8;
        buf[2] = len & 0xFF;
        mqtt_publish(client, "$dp", buf, len + 3, 0, 0);

		for (int i = 0 ; i < len + 3; i ++){
			printf("0x%02x ", buf[i]);
		}
		printf(", len:%d\n", len+3);
    
        vTaskDelay((unsigned long long)ONENET_PUB_INTERVAL* 1000 / portTICK_RATE_MS);
   }
}


void onenet_start(mqtt_client *client)
{
    if(!onenet_initialised) {
        xTaskCreate(&onenet_task, "onenet_task", 2048, client, CONFIG_MQTT_PRIORITY + 1, &xOneNetTask);
        onenet_initialised = true;
    }
}

void onenet_stop(mqtt_client *client)
{
    if(onenet_initialised) {
        if(xOneNetTask) {
            vTaskDelete(xOneNetTask);
        }
        onenet_initialised = false;
    }
}

void connected_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;
    //mqtt_subscribe(client, "/test", 0);
    //mqtt_publish(client, "/test", "howdy!", 6, 0, 0);
    onenet_start(client);
}
void disconnected_cb(void *self, void *params)
{
     mqtt_client *client = (mqtt_client *)self;
     onenet_stop(client);
}
void reconnect_cb(void *self, void *params)
{
    mqtt_client *client = (mqtt_client *)self;
    onenet_start(client);
}
void subscribe_cb(void *self, void *params)
{
    INFO("[APP] Subscribe ok, test publish msg\n");
    mqtt_client *client = (mqtt_client *)self;
    //mqtt_publish(client, "/test", "abcde", 5, 0, 0);
}

void publish_cb(void *self, void *params)
{

}
void data_cb(void *self, void *params)
{
    (void)self;
    mqtt_event_data_t *event_data = (mqtt_event_data_t *)params;

    if (event_data->data_offset == 0) {

        char *topic = malloc(event_data->topic_length + 1);
        memcpy(topic, event_data->topic, event_data->topic_length);
        topic[event_data->topic_length] = 0;
 //       INFO("[APP] Publish topic: %s\n", topic);
        free(topic);
    }

    char *buff = malloc(event_data->data_length + 1);
         memcpy(buff, event_data->data, event_data->data_length);
         printf("received_data: ");
         for(int i=0;i<event_data->data_length + event_data->data_offset;i++)
         {
            //buff[i]=event_data->data_length + event_data->data_offset
			printf("0x%02x ", buff[i]);
            free_data[i]=buff[i];
            free_signal[1]=i+1;
		}        
        free_signal[0]=0;
        printf("\n");
       // printf("finish\n");
        free(buff);

    // char *data = malloc(event_data->data_length + 1);
    // memcpy(data, event_data->data, event_data->data_length);
    // data[event_data->data_length] = 0;
    // data);
             
    // free(data);
//    INFO("[APP] Publish data[%d/%d bytes]\n",
 //        event_data->data_length + event_data->data_offset,
 //        event_data->data_total_length);


}

mqtt_settings settings = {
    .host = ONENET_HOST,
    .port = ONENET_PORT,
    .client_id = ONENET_DEVICE_ID,
    .username = ONENET_PROJECT_ID,
    .password = ONENET_AUTH_INFO,
    .clean_session = 0,
    .keepalive = 120,
    .lwt_topic = "/lwt",
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = connected_cb,
    .disconnected_cb = disconnected_cb,
    .reconnect_cb = reconnect_cb,
    .subscribe_cb = subscribe_cb,
    .publish_cb = publish_cb,
    .data_cb = data_cb
};

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{

    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;

    case SYSTEM_EVENT_STA_GOT_IP:

        mqtt_start(&settings);
        // Notice that, all callback will called in mqtt_task
        // All function publish, subscribe
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        
        mqtt_stop();
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    default:
        break;
    }
    return ESP_OK;


}

void wifi_conn_init(void)
{
    INFO("[APP] Start, connect to Wifi network: %s ..\n", WIFI_SSID);

    tcpip_adapter_init();

    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&icfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        },
    };

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK( esp_wifi_start());
}

static void uart2_tx_task()
{
    static const char *UART2_TX_TASK_TAG = "UART2_TX_TASK";
    esp_log_level_set(UART2_TX_TASK_TAG, ESP_LOG_INFO);
    while (1) 
    {
        if(free_signal[0]==1)
        {
  //      sendDataMoto(ECHO_UART2_PORT,moto_move,1,2048,9);
  //      vTaskDelay(2000 / portTICK_PERIOD_MS);
  //      sendDataMoto(ECHO_UART2_PORT,moto_move,1,3554,9);
        INFO("[APP] send_Data %d\n",free_signal[0]);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        else if(free_signal[0]==0)
        {
        INFO("[APP] send_Data %d\n",free_signal[0]);
        sendDataMoto(ECHO_UART2_PORT,free_data,free_signal[0],2048,free_signal[1]);
        }
    }
}

void uart2_init() {
    const int uart2_num = ECHO_UART2_PORT;
    uart_config_t uart2_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };

    static const char *TAG = "UART2_ECHO_APP";
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Start UART2 application test and configure UART.");

    uart_param_config(uart2_num, &uart2_config);
    uart_set_pin(uart2_num, UART2_TXD_PIN, UART2_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(uart2_num, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Set RS485 half duplex mode
    uart_set_mode(uart2_num, UART_MODE_RS485_HALF_DUPLEX);

    //ESP_LOGI(TAG, "UART start recieve loop.\r\n");
    uart_write_bytes(uart2_num, "Start UART2 test.\r\n", 24);
}

void app_main()
{
    INFO("[APP] Startup..\n");
    INFO("[APP] Free memory: %d bytes\n", esp_get_free_heap_size());
    INFO("[APP] Build time: %s\n", BUID_TIME);

#ifdef CPU_FREQ_160MHZ
    INFO("[APP] Setup CPU run as 160MHz\n");
    SET_PERI_REG_BITS(RTC_CLK_CONF, RTC_CNTL_SOC_CLK_SEL, 0x1, RTC_CNTL_SOC_CLK_SEL_S);
    WRITE_PERI_REG(CPU_PER_CONF_REG, 0x01);
    INFO("[APP] Setup CPU run as 160MHz - Done\n");
#endif
 
    nvs_flash_init();
    wifi_conn_init();
    uart2_init();
    xTaskCreate(uart2_tx_task, "uart2_tx_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);
}
