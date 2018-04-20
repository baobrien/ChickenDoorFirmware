/*
Copyright (c) 2017, Brady O'Brien
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

//#include "SparkFunPhant.h"
//#include "Adafruit_Sensor.h"
//#include "Adafruit_BME280.h"
#include "sunriset.h"
#include "time.h"
#include <cstring>
#include <cstdio>
#include "DHT.h"

/* const char phantServer[] = "data.sparkfun.com"; // Phant destination server
const char publicKey[] = "5YMvr9GlyxtrKJMwYdZj"; // Phant public key
const char privateKey[] = "72n41zqW7xt7Kzxw4q1Y"; // Phant private key
Phant phant(phantServer, publicKey, privateKey); // Create a Phant object*/

 const int doorOpenButton = D3;
 const int doorCloseButton = D2;
 const int doorOpenRelay = D4;
 const int doorCloseRelay = D5;
 const int lightSensor = A0;

int lightSensorReading;
double tempReading;
double humidityReading;
double pressureReading;
String doorStatus;
volatile int checkingSensorState;


const int doorButtonTime = 15*60;

SYSTEM_THREAD(ENABLED);



//Timer doorAnimationTimer(500,doorStateAnimation);

/*LEDStatus statusDoorSensor     (0x000050, LED_PATTERN_SOLID, 1000             , LED_PRIORITY_NORMAL);
LEDStatus statusDoorOpenTimer  (0x00FF00, LED_PATTERN_BLINK, LED_SPEED_NORMAL , LED_PRIORITY_NORMAL);
LEDStatus statusDoorOpen       (0x00FF00, LED_PATTERN_FADE , LED_SPEED_SLOW   , LED_PRIORITY_NORMAL);
LEDStatus statusDoorClosedTimer(0xFF0000, LED_PATTERN_BLINK, LED_SPEED_NORMAL , LED_PRIORITY_NORMAL);
LEDStatus statusDoorClosed     (0xFF0000, LED_PATTERN_FADE , LED_SPEED_SLOW   , LED_PRIORITY_NORMAL);

void updateLedStatus(){
  int onTimer = (doorTimeChange>0);
  switch(currentDoorState){
    DOOR_F_OPEN:
      statusDoorSensor     .setActive(false);
      statusDoorOpenTimer  .setActive(false);
      statusDoorOpen       .setActive(true);
      statusDoorClosedTimer.setActive(false);
      statusDoorClosed     .setActive(false);
    break;
    DOOR_F_CLOSE:
      statusDoorSensor     .setActive(false);
      statusDoorOpenTimer  .setActive(false);
      statusDoorOpen       .setActive(false);
      statusDoorClosedTimer.setActive(false);
      statusDoorClosed     .setActive(true);
    break;
    DOOR_SENSOR:
    default:
      statusDoorSensor     .setActive(true);
      statusDoorOpenTimer  .setActive(false);
      statusDoorOpen       .setActive(false);
      statusDoorClosedTimer.setActive(false);
      statusDoorClosed     .setActive(false);
    break;
  }
  //statusDoorSensor     .setActive(true);
  //statusDoorOpenTimer  .setActive(true);
  //statusDoorOpen       .setActive(true);

  statusDoorClosedTimer.setActive(false);
  statusDoorClosedTimer.setActive(true);
  //statusDoorClosed     .setActive(true);
  switch(currentDoorState){
      DOOR_F_OPEN:
        statusDoorSensor     .off();
        statusDoorOpenTimer  .off();
        statusDoorOpen       .on();
        statusDoorClosedTimer.off();
        statusDoorClosed     .off();
      break;
      DOOR_F_CLOSE:
        statusDoorSensor     .off();
        statusDoorOpenTimer  .off();
        statusDoorOpen       .off();
        statusDoorClosedTimer.off();
        statusDoorClosed     .on();
      break;
      DOOR_SENSOR:
      default:
        statusDoorSensor     .on();
        statusDoorOpenTimer  .off();
        statusDoorOpen       .off();
        statusDoorClosedTimer.off();
        statusDoorClosed     .off();
      break;
    }
}*/


const double lat = 38.927110;
const double lon = -89.7474;
//Time after sunup and sundown to open/close door
const time_t sunrise_offset = 0;
const time_t sunset_offset = 0;
const double sun_angle = -3.0;

bool isSunUp(){
    int year,month,day,hour,min,sec;
    double rise, set;
    
    time_t urise, uset;
    
    auto current = (time_t) Time.now();
    
    struct tm cttm;
    gmtime_r(&current,&cttm);
    
    year = cttm.tm_year + 1900;
    month = cttm.tm_mon + 1;
    day = cttm.tm_mday;
    hour = cttm.tm_hour;
    min = cttm.tm_min;
    sec = cttm.tm_sec;
    
    /* Go one day forward and back to account for UTC offset */
    for( int i = -2; i<=2; i++){
        //sun_rise_set(year, month, day, lon, lat, &rise, &set);
        //bird_twilight(year, month, day + i, lon, lat, &rise, &set);
        __sunriset__( year, month, day, lon, lat, sun_angle, 0, &rise, &set );
        
        time_t day_start = current - (hour*3600 + min*60 + sec) + i*24*60*60;
        
        urise = day_start + (time_t)(rise*3600) + sunrise_offset;
        uset =  day_start + (time_t)(set*3600) + sunset_offset;

        
        if(current > urise && current < uset){
            return true;
        }
    }
    return false;
}

ApplicationWatchdog doorUpdateWd(60000, System.reset);

enum eDoorCtl               { DOOR_F_OPEN = 0, DOOR_F_CLOSE = 1, DOOR_SENSOR = 2};
enum eDoorAutoMode          { AM_SENSOR = 0, AM_SUNRISESET = 1, AM_MANUAL = 2};
enum eDoorOverrideTimeout   { OVR_TIMEOUT_TIME = 0, OVR_TIMEOUT_SUN = 1, OVR_TIMEOUT_NONE = 2};

eDoorCtl                        doorCtl;
eDoorAutoMode                   doorAutoMode;
eDoorCtl                        doorOverrideCtl;
eDoorOverrideTimeout            doorOverrideTimeout;
bool                            doorOverrideEnabled;
bool                            lastSunState;

time_t overrideTimeoutTime;



void checkDoorState(){
    doorUpdateWd.checkin();
    char buf[100];
    time_t delt = (time_t)Time.now() - overrideTimeoutTime;
    snprintf(buf, 99,"%d,%d,%d,%d,%d,%d,%d",
             (int)doorCtl,
             (int)doorAutoMode,
             (int)doorOverrideCtl,
             (int)doorOverrideTimeout,
             (int)doorOverrideEnabled,
             (int)lastSunState,
             (int)delt);
    doorStatus = String(buf);
    
    if(checkingSensorState){
        return;
    }
    time_t now = (time_t)Time.now();
    bool sunUp = isSunUp();
    
    if(doorOverrideEnabled){
        doorCtl = doorOverrideCtl;
        
        switch(doorOverrideTimeout){
            case OVR_TIMEOUT_TIME:
                if(overrideTimeoutTime < now){
                    doorOverrideEnabled = false;
                }
                break;
                
            case OVR_TIMEOUT_SUN:
                if(sunUp != lastSunState){
                    doorOverrideEnabled = false;
                }
                break;
            
            case OVR_TIMEOUT_NONE:
            default:
                break;
        }
    }else{
        switch(doorAutoMode){
            case AM_SUNRISESET:
                if(sunUp){
                    doorCtl = DOOR_F_OPEN;
                }else{
                    doorCtl = DOOR_F_CLOSE;
                }
                break;
            case AM_SENSOR:
            case AM_MANUAL:
                break;
            default:
                doorAutoMode = AM_SUNRISESET;
        }
    }
    switch(doorCtl){
        case DOOR_F_OPEN:
        digitalWrite(doorOpenRelay,HIGH);
        digitalWrite(doorCloseRelay,LOW);
        break;
        case DOOR_F_CLOSE:
        digitalWrite(doorCloseRelay,HIGH);
        digitalWrite(doorOpenRelay,LOW);
        break;
        case DOOR_SENSOR:
        default:
        digitalWrite(doorOpenRelay,LOW);
        digitalWrite(doorCloseRelay,LOW);
    }
    lastSunState = sunUp;
    
}

/*void checkDoorState(){
    unsigned long currentTime = millis();
    int reset = 0;

    if(checkingSensorState)
        return;
    //Check for millis overflow!
    if(currentTime < lastStateCheckTime)
        reset = 1;
        if(reset && doorTimeChange > 0){
            currentDoorState = DOOR_SENSOR;
            doorTimeChange = -1;
        }
    if(currentTime > doorTimeChange)
        reset = 1;
    
    switch(doorAutoMode){
        DOOR_MANUAL:
        DOOR_AUTO_SENSOR:
            if(reset && doorTimeChange > 0){
                currentDoorState = DOOR_SENSOR;
                doorTimeChange = -1;
            }
        default:
            doorAutoMode = DOOR_AUTO_SUNRISESET;
            break;
        DOOR_AUTO_SUNRISESET:
    }
    switch(currentDoorState){
        case DOOR_F_OPEN:
        digitalWrite(doorOpenRelay,HIGH);
        digitalWrite(doorCloseRelay,LOW);
        break;
        case DOOR_F_CLOSE:
        digitalWrite(doorCloseRelay,HIGH);
        digitalWrite(doorOpenRelay,LOW);
        break;
        case DOOR_SENSOR:
        default:
        digitalWrite(doorOpenRelay,LOW);
        digitalWrite(doorCloseRelay,LOW);
    }
    lastStateCheckTime = currentTime;
    //updateLedStatus();
}*/

Timer doorCheck(500,checkDoorState);

//Adafruit_BME280 bme;

//Conversion factor from Pa to inHg

DHT dht_sensor(A4,DHT11);
double pa_to_inhg = 0.2953/1000;

void updateTempHumidityReading(){
    double tempSum = 0;
    double humidSum = 0;
    double pressureSum = 0;
    const auto count = 15;
    for(auto i=0; i<count; i++){
        tempSum += dht_sensor.readTemperature();
        humidSum += dht_sensor.readHumidity();
        //pressureSum += bme.readPressure();
    }
    if(isnan(tempSum)) tempSum = -100*count;
    if(isnan(humidSum)) humidSum = -100*count;
    tempReading = tempSum / (double)count;
    humidityReading = humidSum / (double)count;
    pressureReading = (pressureSum / (double)count) * pa_to_inhg;
}

void sensorUpdate(){
  updateTempHumidityReading();
}

Timer sensorUpdateTimer(1000*60,sensorUpdate);

int doLightSensorRead(String arg){
  sensorUpdate();
  return lightSensorReading;
}

const int override_max_time = 60*60;

void setDoorOverride(eDoorCtl overrideMode, int overrideTime){
    doorOverrideCtl = overrideMode;
    doorOverrideEnabled = 1;
    if(overrideTime >= 0){
        if(overrideTime >= override_max_time)
            overrideTime = override_max_time;
        doorOverrideTimeout = OVR_TIMEOUT_TIME;
        time_t current = (time_t)Time.now();
        overrideTimeoutTime = current + overrideTime;
    }else{
        doorOverrideTimeout = OVR_TIMEOUT_SUN;
    }
    
}

int resetDoor(String param){
    doorOverrideEnabled = 0;
    checkDoorState();
    return 0;
}

int forceOpenDoorTime(String param){
    int doorTime = param.toInt();
    // This is a semantically meaningless check. It's here to remind you that an invalid toInt() will return zero
    // but that's okay
    if(doorTime == 0){
        doorTime = -1;
    }
    setDoorOverride(DOOR_F_OPEN,doorTime);
    checkDoorState();
    return doorTime;
}

int forceCloseDoorTime(String param){
    int doorTime = param.toInt();
    // This is a semantically meaningless check. It's here to remind you that an invalid toInt() will return zero
    // but that's okay
    if(doorTime == 0){
        doorTime = -1;
    }
    setDoorOverride(DOOR_F_CLOSE,doorTime);
    checkDoorState();
    return doorTime;
}

STARTUP(WiFi.selectAntenna(ANT_AUTO));

const int tzoffset = -6;

void configureTimezoneDST(){
    Time.zone(tzoffset);
}


void setup() {
    // Put initialization like pinMode and begin functions here.
    pinMode(doorOpenButton,INPUT);
    pinMode(doorCloseButton,INPUT);
    pinMode(doorOpenRelay,OUTPUT);
    pinMode(doorCloseRelay,OUTPUT);
    //bme.begin(0x76);

    lightSensorReading = analogRead(lightSensor);
    checkingSensorState = 0;
    //updateLightSensorReading();
    updateTempHumidityReading();
    
    doorCtl = DOOR_F_CLOSE;
    doorAutoMode = AM_SUNRISESET;
    doorOverrideCtl = DOOR_SENSOR;
    doorOverrideEnabled = false;
    lastSunState = false;

    configureTimezoneDST();
    
    doorUpdateWd.checkin();
    doorCheck.start();
    sensorUpdateTimer.start();
    //doorAnimationTimer.start();
    //RGB.control(true);
    
    
    Particle.variable("lightsensor",lightSensorReading);
    Particle.variable("pressure",pressureReading);
    Particle.variable("humidity",humidityReading);
    Particle.variable("temperature",tempReading);
    Particle.variable("doorstatus",doorStatus);
    
    Particle.function("ResetDoor",resetDoor);
    Particle.function("OpenTime",forceOpenDoorTime);
    Particle.function("CloseTime",forceCloseDoorTime);
    Particle.function("readLight",doLightSensorRead);
    Particle.publish("setup-done");

}

void onOpenButton(){
}

void onCloseButton(){
}

int closeButtonLast = 0;
int openButtonLast = 0;
// loop() runs over and over again, as quickly as it can execute.
void loop() {
  int closeButton = !digitalRead(doorCloseButton);
  int openButton = !digitalRead(doorOpenButton);

  if(openButton){
    onOpenButton();
  }

  if(closeButton){
    onCloseButton();
  }

}

