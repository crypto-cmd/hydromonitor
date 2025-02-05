//#############################################################################################
//#                                                                                           #
//#                                      NTP HEADER FILE                                      #
//#                                                                                           #
//#############################################################################################

/*
          GOAL
1. Maintain accurate date and time regardless of internet connectivity

          TASKS
1. Get current date and time from NTP server and then adjust ESP32's system time, every minute once there is internet connectivity
2. Update system time from SPIFFS after a restart
3. Update SPIFFS from system time every minute
4. Publish timestampUpdate to PI every minute once PI is ON
5. Print date and time every minute

*/

#ifndef _NTP_H
#define _NTP_H 1
#endif

#ifndef _WIFI_H 
#include <WiFi.h>
#endif

#ifndef _TIME_H 
#include "time.h"
#endif

#ifndef _SNTP_H 
#include "sntp.h"
#endif


// HELP FULL WEBPAGES
// https://www.tutorialspoint.com/c_standard_library/c_function_strftime.htm
// https://pubs.opengroup.org/onlinepubs/009695399/basedefs/time.h.html
// https://pubs.opengroup.org/onlinepubs/9699919799/    Search usine keyword "time" in searchbox
// https://www.epochconverter.com/programming/c
// https://www.timeanddate.com/worldclock/jamaica/kingston
// https://pubs.opengroup.org/onlinepubs/7908799/xsh/systime.h.html
// https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
// https://forum.arduino.cc/t/esp32-settimeofday-functionality-giving-odd-results/676136

class Ntp {
  
  private:
          const char* ntpServer           = "north-america.pool.ntp.org";  //"pool.ntp.org";
          const char* ntpServer1          = "pool.ntp.org";
          const char* ntpServer2          = "time.nist.gov";
          const long  gmtOffset_sec       = -5*3600;
          const int   daylightOffset_sec  = 0;
          int         currenttime         = 0;
          const char* time_zone           = "EST+5EDT,M3.2.0/2,M11.1.0/2"; //"CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)
          

  public:
        unsigned long epochTime;
        Ntp(const char* process){ 
          Serial.println(process); 
        }
        
        static void printLocalTime(){
            struct tm timeinfo;
            if(!getLocalTime(&timeinfo)){
              Serial.println("No time available (yet)");
              return;
            }
            // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
             Serial.println(&timeinfo, "%B %d, %Y    %I:%M %p");  
        }

        // Callback function (get's called when time adjusts via NTP)
        static void timeavailable(struct timeval *t) {
          Serial.println("Got time adjustment from NTP!");
          printLocalTime();
        }
          
        unsigned long getTime(){
          // RETURNS TIMESTAMP FOR CURRENT TIME FROM NTP SERVER
              time_t now;
              struct tm timeinfo;

              // get time from ntp server first and save to system
              if (!getLocalTime(&timeinfo)) {
                Serial.println("Failed to obtain time, NTP");
                return(0);
              }
              // Serial.println(&timeinfo, "%B %d, %Y    %I:%M %p");
              // Retrieve time[Timestamp] from system and save to &now variable
              time(&now); 
              return now;  
        } 
 
        unsigned long getTimeStamp(void){
            // RETURNS 10 DIGIT TIMESTAMP REPRESENTING CURRENT TIME
            time_t now;  
            time(&now); // Retrieve time[Timestamp] from system and save to &now variable
            return now;
        }

        void setup(){
            // set notification call-back function
            sntp_set_time_sync_notification_cb( timeavailable );
            /**
              * NTP server address could be aquired via DHCP,
              *
              * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
              * otherwise SNTP option 42 would be rejected by default.
              * NOTE: configTime() function call if made AFTER DHCP-client run
              * will OVERRIDE aquired NTP server address
              */
            sntp_servermode_dhcp(1);    // (optional)
            /**
            * This will set configured ntp servers and constant TimeZone/daylightOffset
            * should be OK if your time zone does not need to adjust daylightOffset twice a year,
            * in such a case time adjustment won't be handled automagicaly.
            */
            // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);
            configTzTime(this->time_zone, ntpServer1, ntpServer2);              
        } 


};



// NTP TASK 
void vNTP( void * pvParameters ) {
  configASSERT( ( ( uint32_t ) pvParameters ) == 1 );   
  Ntp NTP = Ntp("NTP PROTOCOL INITIATED"); // Instatiate NTP Class
  NTP.setup(); // Config NTP
 
 for( ;; ) {  
      // NTP.printLocalTime();     // it will take some time to sync time :)  
      // NTP.getTime();
      // checkHEAP("NTP");   
      vTaskDelay(60000 / portTICK_PERIOD_MS);  // DELAY FOR 5 SECONDS  
    }     
 }



// Function that creates a task.
void vNTPFunction( void ) {
    BaseType_t xReturned;

    // Create the task, storing the handle. 
    xReturned = xTaskCreatePinnedToCore(
                    vNTP,               // Function that implements the task. 
                    "NTP Protocol",     // Text name for the task. 
                    1500,               // Stack size (Bytes in ESP32, words in Vanilla FreeRTOS) 
                    ( void * ) 1,       // Parameter passed into the task. 
                    12,                  // Priority at which the task is created. 
                    &xNTPHandle,        // Used to pass out the created task's handle. 
                    1);                 // ESP Core to run task on. 

    if( xReturned == pdPASS ) {  
      // The task was created.  Use the task's handle to delete the task. 
       Serial.println("NTP PROTOCOL TASK CREATED");
    }
    else {
       Serial.println("UNABLE TO CREATE NTP PROTOCOL TASK");
    }
}