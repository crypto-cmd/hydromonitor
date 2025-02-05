// ##################################################################################################################
// ##                                      ELET2415 DATA ACQUISITION SYSTEM CODE                                   ##
// ##                                                                                                              ##
// ##################################################################################################################

// LIBRARY IMPORTS
#include <rom/rtc.h>
#include <math.h> // https://www.tutorialspoint.com/c_standard_library/math_h.htm
#include <ctype.h>

// ADD YOUR IMPORTS HERE

#ifndef _WIFI_H
#include <WiFi.h>
#endif

#ifndef STDLIB_H
#include <stdlib.h>
#endif

#ifndef STDIO_H
#include <stdio.h>
#endif

#ifndef ARDUINO_H
#include <Arduino.h>
#endif

#ifndef ARDUINOJSON_H
#include <ArduinoJson.h>
#endif

// DEFINE VARIABLES
#define ARDUINOJSON_USE_DOUBLE 1

// DEFINE THE CONTROL PINS FOR THE DHT22

// MQTT CLIENT CONFIG
static const char *pubtopic = "620164974";                      // Add your ID number here
static const char *subtopic[] = {"620164974_sub", "/elet2415"}; // Array of Topics(Strings) to subscribe to
static const char *mqtt_server = "broker.emqx.io";              // Broker IP address or Domain name as a String
static uint16_t mqtt_port = 1883;

// WIFI CREDENTIALS
const char *ssid = "Shane's S24 Ultra"; // Add your Wi-Fi ssid
const char *password = "ijun1234";      // Add your Wi-Fi password

// TASK HANDLES
TaskHandle_t xMQTT_Connect = NULL;
TaskHandle_t xNTPHandle = NULL;
TaskHandle_t xLOOPHandle = NULL;
TaskHandle_t xUpdateHandle = NULL;
TaskHandle_t xButtonCheckeHandle = NULL;

// FUNCTION DECLARATION
void checkHEAP(const char *Name); // RETURN REMAINING HEAP SIZE FOR A TASK
void initMQTT(void);              // CONFIG AND INITIALIZE MQTT PROTOCOL
unsigned long getTimeStamp(void); // GET 10 DIGIT TIMESTAMP FOR CURRENT TIME
void callback(char *topic, byte *payload, unsigned int length);
void initialize(void);
bool publish(const char *topic, const char *payload); // PUBLISH MQTT MESSAGE(PAYLOAD) TO A TOPIC
void vButtonCheck(void *pvParameters);
void vUpdate(void *pvParameters);
bool isNumber(double number);

/* Declare your functions below */
double convert_Celsius_to_fahrenheit(double c);
double convert_fahrenheit_to_Celsius(double f);
double calcHeatIndex(double Temp, double Humid);

/* Init class Instances for the DHT22 etcc */

// ############### IMPORT HEADER FILES ##################
#ifndef NTP_H
#include "NTP.h"
#endif

#ifndef MQTT_H
#include "mqtt.h"
#endif

// Temporary Variables

void setup()
{
  Serial.begin(115200); // INIT SERIAL

  // INITIALIZE ALL SENSORS AND DEVICES

  /* Add all other necessary sensor Initializations and Configurations here */
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  initialize(); // INIT WIFI, MQTT & NTP
                // vButtonCheckFunction(); // UNCOMMENT IF USING BUTTONS INT THIS LAB, THEN ADD LOGIC FOR INTERFACING WITH BUTTONS IN THE vButtonCheck FUNCTION
}

void loop()
{
  // put your main code here, to run repeatedly:
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ####################################################################
// #                          UTIL FUNCTIONS                          #
// ####################################################################
void vButtonCheck(void *pvParameters)
{
  configASSERT(((uint32_t)pvParameters) == 1);

  for (;;)
  {
    // Add code here to check if a button(S) is pressed
    // then execute appropriate function if a button is pressed

    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void vUpdate(void *pvParameters)
{
  configASSERT(((uint32_t)pvParameters) == 1);

  for (;;)
  {
    // #######################################################
    // ## This function must PUBLISH to topic every second. ##
    // #######################################################

    // 1. Read Humidity and save in variable below
    double h = 0;

    // 2. Read temperature as Celsius   and save in variable below
    double t = 0;

    if (isNumber(t))
    {
      // ##Publish update according to ‘{"id": "student_id", "timestamp": 1702212234, "temperature": 30, "humidity":90, "heatindex": 30}’

      // 1. Create JSon object

      // 2. Create message buffer/array to store serialized JSON object

      // 3. Add key:value pairs to JSon object based on above schema

      // 4. Seralize / Covert JSon object to JSon string and store in message array

      // 5. Publish message to a topic sobscribed to by both backend and frontend

      JsonDocument doc; // Create JSon object
      char message[1100] = {0};

      // Add key:value pairs to JSon object
      doc["id"] = "620164974"; // Change to your student ID number
      doc["timestamp"] = getTimeStamp();
      doc["temperature"] = t;
      doc["humidity"] = h;
      doc["heatindex"] = calcHeatIndex(t, h);

      serializeJson(doc, message); // Seralize / Covert JSon object to JSon string and store in char* array

      if (mqtt.connected())
      {
        publish(pubtopic, message);
      } else {
        Serial.println("MQTT not connected. Cannot publish");
      }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

unsigned long getTimeStamp(void)
{
  // RETURNS 10 DIGIT TIMESTAMP REPRESENTING CURRENT TIME
  time_t now;
  time(&now); // Retrieve time[Timestamp] from system and save to &now variable
  return now;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  // ############## MQTT CALLBACK  ######################################
  // RUNS WHENEVER A MESSAGE IS RECEIVED ON A TOPIC SUBSCRIBED TO

  Serial.printf("\nMessage received : ( topic: %s ) \n", topic);
  char *received = new char[length + 1]{0};

  for (int i = 0; i < length; i++)
  {
    received[i] = (char)payload[i];
  }

  // PRINT RECEIVED MESSAGE
  Serial.printf("Payload : %s \n", received);

  // CONVERT MESSAGE TO JSON
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, received);

  if (error)
  {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  // PROCESS MESSAGE
  const char *type = doc["type"];

  if (strcmp(type, "controls") == 0)
  {
    // 1. EXTRACT ALL PARAMETERS: NODES, RED,GREEN, BLUE, AND BRIGHTNESS FROM JSON OBJECT
    const char *nodes = doc["nodes"];
    const char *red = doc["red"];
    const char *green = doc["green"];
    const char *blue = doc["blue"];
    const char *brightness = doc["brightness"];


    // 2. ITERATIVELY, TURN ON LED(s) BASED ON THE VALUE OF NODES. Ex IF NODES = 2, TURN ON 2 LED(s)
    turnOnLEDs(nodes, red, green, blue, brightness);

    // 3. ITERATIVELY, TURN OFF ALL REMAINING LED(s).
    turnOffLEDs(nodes);
  }
}

bool publish(const char *topic, const char *payload)
{
  bool res = false;
  try
  {
    res = mqtt.publish(topic, payload);
    // Serial.printf("\nres : %d\n",res);
    if (!res)
    {
      res = false;
      throw false;
    }
  }
  catch (...)
  {
    Serial.printf("\nError (%d) >> Unable to publish message\n", res);
  }
  return res;
}

//***** Complete the util functions below ******

double convert_Celsius_to_fahrenheit(double c){
    return c * 9 / 5 + 32}

double convert_fahrenheit_to_Celsius(double f){
    return (f - 32) * 5 / 9}

double calcHeatIndex(double Temp, double Humid)
{
  double heatIndex = -42.379 + 2.04901523 * Temp - 10.14333127 * Humid - 0.22475541 * Temp * Humid - 6.83783e-3 * Temp * Temp - 5.481717e-2 * Humid * Humid - 1.22874e-3 * Temp * Temp * Humid + 8.5282e-4 * Temp * Humid * Humid - 1.99e-6 * Temp * Temp * Humid * Humid;
  return heatIndex;
}

bool isNumber(double number)
{
  char item[20];
  snprintf(item, sizeof(item), "%f\n", number);
  if (isdigit(item[0]))
    return true;
  return false;
}