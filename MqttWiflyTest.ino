// WiFly libraries
#include <SPI.h>
#include <WiFly.h>
#include <SoftwareSerial.h>


#include "config.h"

#include <MemoryFree.h>


#if DEBUG
void debug(const __FlashStringHelper * console_text)
{
  Serial.println(console_text);
}
#endif


// callback function definition required here as client needs to be defined before
// including relay.h
void callback(char* topic, uint8_t* payload, unsigned int length);

PubSubClient   mqttClient(mqtt_server_address, MQTT_PORT, callback, wiFlyClient);

void publish_connected()
{
  prog_buffer[0] = '\0';
  strcpy_P(prog_buffer, (char*)pgm_read_word(&(STATUS_TOPICS[0])));
  mqttClient.publish(prog_buffer, "connected");
}

void publish_uptime() {
  prog_buffer[0] = '\0';
  strcpy_P(prog_buffer, (char*)pgm_read_word(&(STATUS_TOPICS[1])));
  char_buffer[0] = '\0';
  ltoa(millis(), char_buffer, 10);
  mqttClient.publish(prog_buffer, char_buffer);

}

void publish_memory() {
  prog_buffer[0] = '\0';
  strcpy_P(prog_buffer, (char*)pgm_read_word(&(STATUS_TOPICS[2])));
  char_buffer[0] = '\0';
  itoa(freeMemory(), char_buffer, 10);
  mqttClient.publish(prog_buffer, char_buffer);
}

void callback(char* topic, uint8_t* payload, unsigned int payload_length) {
  // handle message arrived
  /* topic = part of the variable header:has topic name of the topic where the publish received
   	  NOTE: variable header does not contain the 2 bytes with the
   	        publish msg ID
   	  payload = pointer to the first item of the buffer array that
   	            contains the message tha was published
   	           EXAMPLE of payload: lights,1
   	  length = the length of the payload, until which index of payload
  */

#if DEBUG
  debug(F("Payload length is"));
  Serial.println(payload_length);
#endif

  // Copy the payload to the new buffer
  char* message = (char*)malloc((sizeof(char) * payload_length) + 1); // get the size of the bytes and store in memory
  memcpy(message, payload, payload_length * sizeof(char));        // copy the memory
  message[payload_length * sizeof(char)] = '\0';                  // add terminating character

#if DEBUG
  debug(F("Message with topic"));
  Serial.println(topic);
  debug(F("arrived with payload"));
  Serial.println(message);
#endif

  byte topic_idx = 0;
  // find if topic is matched
  for (byte i = 0; i < ARRAY_SIZE(CONTROL_TOPICS); i++) {
    prog_buffer[0] = '\0';
    strcpy_P(prog_buffer, (PGM_P)pgm_read_word(&(CONTROL_TOPICS[i])));
    if (strcmp(topic, prog_buffer) == 0) {
      topic_idx = i;
      break;
    }
  }
#if DEBUG
  debug(F("Control topic index"));
  Serial.println(topic_idx);
#endif

  if (topic_idx == 0) {  // topic is UPTIME_REQUEST
    publish_uptime();
#if DEBUG
    debug(F("UPTIME_REQUEST topic arrived"));
    Serial.println(millis());
#endif
  } else if (topic_idx == 1) {  // topic is MEMORY_REQUEST
    publish_memory();
#if DEBUG
    debug(F("MEMORY_REQUEST topic arrived"));
    Serial.println(freeMemory());
#endif
  }

  // free memory assigned to message
  free(message);
}

#if 0
void wifly_connect() {
#if DEBUG
  debug(F("initialising wifly"));
#endif

  WiFly.begin();
  delay(5000);  // allow time to WiFly to initialise

#if DEBUG
  debug(F("joining network"));
#endif

  //  if (!WiFly.join(MY_SSID, MY_PASSPHRASE, mode)) {
  if (!WiFly.join(MY_SSID)) {   // needs to be fixed to allow a passphrase if secure
    wiflyConnected = false;
#if DEBUG
    debug(F("  failed"));
#endif
    delay(AFTER_ERROR_DELAY);
  } else {
    wiflyConnected = true;
#if DEBUG
    debug(F("  connected"));
#endif
  }
}

boolean mqtt_connect() {
  if (!wiflyConnected)
    wifly_connect();

  if (wiflyConnected) {
    // MQTT client setup
    //    mqttClient.disconnect();
#if DEBUG
    debug(F("connecting to broker"));
#endif
    if (mqttClient.connect(mqttClientId)) {
#if DEBUG
      debug(F("  connected"));
#endif
      publish_connected();
#if USE_FREEMEM
      publish_memory();
#endif
      // subscribe to topics
      mqttClient.subscribe("relayduino/request/#");
    } else {
#if DEBUG
      debug(F("  failed"));
#endif
      delay(AFTER_ERROR_DELAY);
    }
  }
  return mqttClient.connected();
}

void reset_connection() {
  if (mqttClient.connected())
    mqttClient.disconnect();
  wifly_connect();
  mqtt_connect();
}
#endif

/*
 * method to make sure we have a connection to the Wifi and MQTT server
 * source: https://github.com/freakent/smart_star/blob/master/smart_star.ino
 *
 */

unsigned long connect_alarm; // time of next connection check;

void mqtt_connect() {
    Serial.print("Reset any previous mqtt connection...");
    //client.disconnect();
    Serial.println("OK");
    Serial.print("Connecting to MQTT Broker...");
    if (mqttClient.connect(mqtt_client_id)) {
      Serial.println("OK");
      publish_connected();
      mqttClient.subscribe("relayduino/request/#");
    } else {
      Serial.println("Failed");
    }
}

void ensure_connected() {
  connect_alarm = millis() + 30000UL;
  
  if (!mqttClient.connected()) {
    
 //   light_pattern(0); // turn lights off temporarily just in case this hangs
    
//    if (!wiFlyClient.connected()) {
    if (!wifly_connected) {
      Serial.print("Reset any previous Wifi connection...");
      wiFlyClient.stop();
      Serial.println("OK");
  
      Serial.print("Connecting to WiFi...");
      WiFly.begin();
  
      // Join the WiFi network
//      if (!WiFly.join(ssid, passphrase, mode)) {
      if (!WiFly.join(MY_SSID)) {
        Serial.println(" Failed");
        wifly_connected = false;  
        return;    } 
  
      wifly_connected = true;
      Serial.println(" OK");
    }

    mqtt_connect();    
  }
}


/*--------------------------------------------------------------------------------------
  setup()
  Called by the Arduino framework once, before the main loop begins
  --------------------------------------------------------------------------------------*/
void setup()
{
  // Configure WiFly
  Serial.begin(BAUD_RATE);

  wiFlySerial.begin(BAUD_RATE);
  WiFly.setUart(&wiFlySerial);

//  wifly_connect();
  ensure_connected();

#if DEBUG
  Serial.println(WiFly.ip());
  //  Serial.println(WiFly.getMAC());
#endif

  last_reconnect_attempt = 0;

#if USE_HARDWARE_WATCHDOG
  ResetWatchdog1();
#endif
}


/*--------------------------------------------------------------------------------------
  loop()
  Arduino main loop
  --------------------------------------------------------------------------------------*/
void loop()
{
  // require a client.loop in order to receive subscriptions
//  mqttClient.loop();
//
//  if (!mqttClient.loop()) {
//    mqtt_connect();
//   }

  // alternative based on code in relayr
//  if (mqttClient.connected()) {
//    mqttClient.loop();
//  } else {
//    //if connection lost, try to reconnect
//    mqtt_connect();
//  }

#if 0
  // alternative based on code in pubsubclient example mqtt_reconnect_nonblocking
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - last_reconnect_attempt > 5000) {
      last_reconnect_attempt = now;
      // Attempt to reconnect
      if (mqttClient()) {
        last_reconnect_attempt = 0;
      }
    }
  } else {
    // Client connected
    mqttClient.loop();
  }
#endif

  // alternative based on code at
  // https://github.com/freakent/smart_star/blob/master/smart_star.ino
  if (millis() > connect_alarm) {
    ensure_connected(); 
  }
  
  mqttClient.loop();


#if USE_HARDWARE_WATCHDOG
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= watchdog_interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    ResetWatchdog1();
  }
#endif
}

