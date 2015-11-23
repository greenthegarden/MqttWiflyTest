#ifndef MQTTWIFLYRELAYDUINO_WATCHDOG_CONFIG_H_
#define MQTTWIFLYRELAYDUINO_WATCHDOG_CONFIG_H_


#define USE_HARDWARE_WATCHDOG       false


#if USE_HARDWARE_WATCHDOG

const int RESET_WATCHDOG1         = 10;
unsigned long previous_millis     = 0;              // will store last time LED was updated
const long watchdog_interval      = 60UL * 1000UL;  // interval at which to blink (milliseconds)

void reset_watchdog_1()
{
 pinMode(RESET_WATCHDOG1, OUTPUT);
 delay(200);
 pinMode(RESET_WATCHDOG1, INPUT);
#if DEBUG
 Serial.println("Watchdog1 Reset");
#endif
}

#endif


#endif  /* MQTTWIFLYRELAYDUINO_WATCHDOG_CONFIG_H_ */

