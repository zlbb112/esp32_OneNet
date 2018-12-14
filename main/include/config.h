#ifndef __ONENET_CONFIG_H__
#define __ONENET_CONFIG_H__

//#define CPU_FREQ_160MHZ

/* Here needs to be changed according to your envirnoment. */
#define WIFI_SSID           "lejurobot"
#define WIFI_PASS           "leju20181818"

#define ONENET_HOST         "183.230.40.39"
#define ONENET_PORT         (6002)

/* Here needs to be changed accoding to your oneONET configure. */
#define ONENET_DEVICE_ID    "507271443"                // mqtt client id
#define ONENET_PROJECT_ID   "196992"               // mqtt username
#define ONENET_AUTH_INFO    "11020"				   // mqtt password

/* Here needs to be changed accoding to your oneONET configure. */
#define ONENET_DATA_STREAM  "esp32MQTT"

#define ONENET_PUB_INTERVAL (60) // unit: s

#endif

