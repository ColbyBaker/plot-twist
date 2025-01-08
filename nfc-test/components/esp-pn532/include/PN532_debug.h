#ifndef __DEBUG_H__
#define __DEBUG_H__

#define DEBUG

// #include "Arduino.h"

#include "esp_log.h"
#define TAG "debug"

#ifdef DEBUG
// #define DMSG(args...)       Serial.print(args)
// #define DMSG_STR(str)       Serial.println(str)
// #define DMSG_HEX(num)       Serial.print(' '); Serial.print((num>>4)&0x0F, HEX); Serial.print(num&0x0F, HEX)
// #define DMSG_INT(num)       Serial.print(' '); Serial.print(num)

// #define DMSG(args...)       ESP_LOGI(TAG, args)
// #define DMSG_STR(str)       ESP_LOGI(TAG, "%s", str)
// #define DMSG_HEX(num)       ESP_LOGI(TAG, " %02x", num)
// #define DMSG_INT(num)       ESP_LOGI(TAG, " %d", num)

#define DMSG(args...)       do{}while(0)
#define DMSG_STR(str)       do{}while(0)
#define DMSG_HEX(num)       do{}while(0)
#define DMSG_INT(num)       do{}while(0)

#else
#define DMSG(args...)
#define DMSG_STR(str)
#define DMSG_HEX(num)
#define DMSG_INT(num)
#endif

#endif