#ifndef MQTTWIFLYRELAYDUINO_CONFIG_H_
#define MQTTWIFLYRELAYDUINO_CONFIG_H_


#define DEBUG                       true
#define USE_FREEMEM                 true


// Macros
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


// Serial parameters
const int BAUD_RATE               = 9600;


// Watchdog configuration
#include "Watchdog_config.h"


// Network configuration
#include "networkConfig.h"


// WiFly configuration
#include "wifly_config.h"


// MQTT configuration
#include "mqtt_config.h"


const char COMMAND_SEPARATOR      = ':';

const byte BUFFER_SIZE            = 32;
char char_buffer[BUFFER_SIZE];
char prog_buffer[BUFFER_SIZE];
char message[BUFFER_SIZE];


#endif  /* MQTTWIFLYRELAYDUINO_CONFIG_H_ */

