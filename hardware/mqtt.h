// #############################################################################################
// #                                                                                           #
// #                                      MQTT HEADER FILE                                     #
// #                                                                                           #
// #############################################################################################

#ifndef _MQTT_H
#define _MQTT_H 1
#endif

#include <PubSubClient.h>

// NOTES:
// https://pubsubclient.knolleary.net/api

// ############ VARIABLES ################
static char deviceName[50] = "H"; // HARDWARE

// ############## FUNCTIONS  ####################
void vButtonCheckFunction(void);
void vUpdateFunction(void);

/* create an instance of PubSubClient client */
WiFiClient espClient;
PubSubClient mqtt(espClient);

// ###############################################
// #        DO NOT MODIFY ANY CODE BELOW         #
// ###############################################

//*********************************************************************************************
//                                       MQTT SETUP                                           *
//*********************************************************************************************
void MQTT_Connect(void *pvParameters)
{
  /* The parameter value is expected to be 1 as 1 is passed in the pvParameters value in the call to xTaskCreate() below. */
  configASSERT(((uint32_t)pvParameters) == 1);

  for (;;)
  {

    vTaskDelay(10000 / portTICK_PERIOD_MS);
    // Serial.printf("mqtt connected %d  %d\n",mqtt.connected(),mqtt.state());

    while (!mqtt.connected())
    {

      char clientID[50] = {0};
      char clientName[50] = {0};

      /* Intializes random number generator */
      time_t t;
      srand((unsigned)time(&t));

      // Client ID
      snprintf(clientID, sizeof(clientID), "IOT_%s_%d", deviceName, rand() % 5000);
      Serial.printf("\nMQTT Connection ID: %s ", clientID);

      /* connect now */
      if (mqtt.connect(clientID))
      {
        Serial.println("\n\n***** MQTT CONNECTED! *****\n\n");

        const uint8_t size = sizeof(subtopic) / sizeof(subtopic[0]);
        for (int x = 0; x < size; x++)
        {
          mqtt.subscribe(subtopic[x]);
        }
        break;
      }
      else
      {
        Serial.printf("\nConnection failed with status code : %d ,  re-trying in 10 seconds\n", mqtt.state());
      }

      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
  }
}

/* Function that creates a task. */
void MQTT_ConnectFunction(void)
{
  BaseType_t xReturned;

  /* Create the task, storing the handle. */
  xReturned = xTaskCreatePinnedToCore(
      MQTT_Connect,   /* Function that implements the task. */
      "MQTT CONNECT", /* Text name for the task. */
      8096,           /* Stack size (Bytes in ESP32, words in Vanilla FreeRTOS) */
      (void *)1,      /* Parameter passed into the task. */
      8,              /* Priority at which the task is created. */
      &xMQTT_Connect, /* Used to pass out the created task's handle. */
      1);             /* ESP Core to run task on. */

  if (xReturned == pdPASS)
  {
    /* The task was created.  Use the task's handle to delete the task. */
    Serial.println(" MQTT CONNECT TASK CREATED");
    // vTaskSuspend(xMQTT_Connect ); // SUSPEND UNTIL WIFI CONNECTION IS ESTABLISHED
  }
  else
  {
    Serial.println(" UNABLE TO CREATE MQTT CONNECT TASK");
  }
}

void vLOOP(void *pvParameters)
{
  configASSERT(((uint32_t)pvParameters) == 1);
  //  checkHEAP("vLOOP-BEGIN");

  for (;;)
  {
    // Task code goes here.
    mqtt.loop(); // mqtt MQTT CONNECTION LOOP
    // checkHEAP("vLOOP-BEGIN");
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

// Function that creates a task.
void vLOOPFunction(void)
{
  BaseType_t xReturned;

  // Create the task, storing the handle.
  xReturned = xTaskCreatePinnedToCore(
      vLOOP,        // Function that implements the task.
      "vLOOP",      // Text name for the task.
      8096,         // Stack size (Bytes in ESP32, words in Vanilla FreeRTOS)
      (void *)1,    // Parameter passed into the task.
      15,           // Priority at which the task is created.
      &xLOOPHandle, // Used to pass out the created task's handle.
      0);           // ESP Core to run task on.

  if (xReturned == pdPASS)
  {
    // The task was created.  Use the task's handle to delete the task.
    Serial.println(" vLOOP TASK CREATED");
  }
  else
  {
    Serial.println("UNABLE TO CREATE vLOOP TASK");
  }
}

void initMQTT(void)
{

  Serial.printf("\nMQTT Server : %s   PORT : %d \n", mqtt_server, mqtt_port);
  mqtt.setServer(mqtt_server, mqtt_port); // Configure the MQTT server with IPaddress and port    stationInfo.remoteMqttPort
  mqtt.setCallback(callback);             // This function will be invoked when client received subscribed topic

  mqtt.setBufferSize(2000);
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(15);

  MQTT_ConnectFunction();
  vLOOPFunction();
}

//********************************************************************************************
//                                      MQTT SETUP ENDS                                      *
//********************************************************************************************

// ######################## UTIL FUNCTIONS ###########################
// Stack size (Bytes in ESP32, words in Vanilla FreeRTOS)
void checkHEAP(const char *Name)
{
  // TOTAL AVAILABLE HEAP IN BYTES FOR ESP32 AND THE AMOUNT OF BYTES LEFT IN A SPECIFIC TASK'S STACK
  Serial.print("\n ==>  ");
  Serial.print(Name);
  Serial.print(" HEAP AVAILABLE :  ");
  Serial.print(xPortGetFreeHeapSize());
  Serial.print(" BYTES  ");
  Serial.print("   TASK STACK AVAILABLE ");
  Serial.print(uxTaskGetStackHighWaterMark(NULL) * 4);
  Serial.print(" BYTES  \n");
}

void initialize(void)
{
  vNTPFunction(); // INIT NTP PROTOCOL FOR TIME KEEPING

  // Connect to MQTT
  initMQTT();

  vUpdateFunction();
}

/*
TASK TEMPLATE
void vName( void * pvParameters )  {
      configASSERT( ( ( uint32_t ) pvParameters ) == 1 );
      //checkHEAP("vLOOP-BEGIN");

 for( ;; ) {
     // Task code goes here.

    vTaskDelay(1000 / portTICK_PERIOD_MS);
 }
}

// Function that creates a task.
void vNameFunction( void ) {
   BaseType_t xReturned;

    // Create the task, storing the handle.
    xReturned = xTaskCreatePinnedToCore(
                    vName,               // Function that implements the task.
                    "vName",    // Text name for the task.
                    4096,               // Stack size (Bytes in ESP32, words in Vanilla FreeRTOS)
                    ( void * ) 1,       // Parameter passed into the task.
                    4,                  // Priority at which the task is created.
                    &xNameHandle,        // Used to pass out the created task's handle.
                    1);                 // ESP Core to run task on.

    if( xReturned == pdPASS ){
      // The task was created.  Use the task's handle to delete the task.
      // Serial.println(" vName TASK CREATED");
    }
    else{
      Serial.println("UNABLE TO CREATE vName TASK");
    }
}
*/

// Function that creates a task.
void vButtonCheckFunction(void)
{
  BaseType_t xReturned;

  // Create the task, storing the handle.
  xReturned = xTaskCreatePinnedToCore(
      vButtonCheck,         // Function that implements the task.
      "vButtonCheck",       // Text name for the task.
      4096,                 // Stack size (Bytes in ESP32, words in Vanilla FreeRTOS)
      (void *)1,            // Parameter passed into the task.
      3,                    // Priority at which the task is created.
      &xButtonCheckeHandle, // Used to pass out the created task's handle.
      1);                   // ESP Core to run task on.

  if (xReturned == pdPASS)
  {
    // The task was created.  Use the task's handle to delete the task.
    Serial.println(" vButtonCheck TASK CREATED");
  }
  else
  {
    Serial.println("UNABLE TO CREATE vButtonCheck TASK");
  }
}

// Function that creates a task.
void vUpdateFunction(void)
{
  BaseType_t xReturned;

  // Create the task, storing the handle.
  xReturned = xTaskCreatePinnedToCore(
      vUpdate,        // Function that implements the task.
      "vUpdate",      // Text name for the task.
      4096,           // Stack size (Bytes in ESP32, words in Vanilla FreeRTOS)
      (void *)1,      // Parameter passed into the task.
      6,              // Priority at which the task is created.
      &xUpdateHandle, // Used to pass out the created task's handle.
      1);             // ESP Core to run task on.

  if (xReturned == pdPASS)
  {
    // The task was created.  Use the task's handle to delete the task.
    Serial.println(" vUpdate TASK CREATED");
  }
  else
  {
    Serial.println("UNABLE TO CREATE vUpdate TASK");
  }
}