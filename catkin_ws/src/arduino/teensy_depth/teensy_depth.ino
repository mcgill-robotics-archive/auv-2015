#include <i2c_t3.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#include "defs.h"
#include "MS5803_I2C.h"

MS5803 depthSensor(ADDRESS_HIGH);
ros::NodeHandle nh;

bool depthSensorConnected = false;

unsigned long depthSensorSchedule = 0;

std_msgs::Float32 pressure_m;

ros::Publisher pressurePub("~pressure", &pressure_m);  // Publish the depth topic

void inline toggleLed(){
  digitalWrite(LED_PIN,!digitalRead(LED_PIN));
}

void resetDepthSensor(){
  for(int i = 0; i< 5 ; i++){
    Wire1.resetBus();
    Wire1.beginTransmission(MS5803_I2C_ADDR);
    if(Wire1.endTransmission() == 0){
      depthSensorConnected = true;
      depthSensor.reset();
      depthSensor.begin();
      break;
    } else {
      nh.logwarn("Depth Sensor Unresponsive....");
    }
  }
}

void reconnectDepthSensor(){

  resetDepthSensor(); 

  if(depthSensorConnected){
    nh.logwarn("Depth Sensor Reset Successfully!!!");
  } else {
    nh.logfatal("Depth Sensor Reset has failed!!!");
  }
}

void depthSensorInit(){
  
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire1.setDefaultTimeout(100);
  resetDepthSensor();
  
}

void rosInit(){
  //ros node initialization
  nh.initNode();  
  //ros publisher initialization
  nh.advertise(pressurePub);        //depth sensor
}

void setup(){

  depthSensorInit();
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN, depthSensorConnected);

  rosInit();
}
  
void loop(){
  unsigned long currentTime = millis();
  
  //Depth Sensing
  if(depthSensorSchedule < currentTime){
    if(depthSensorConnected){
      //Get Readings
      depthSensor.getMeasurements(ADC_4096);

      //check status, reconnect if needed
      if(depthSensor.getSensorStatus()){

        nh.logerror("Depth Sensor Communication Error, Attemping Reset...");
        depthSensorConnected = false;

        reconnectDepthSensor();

      } else {
        // passed connection test, putting data to ros
        pressure_m.data = depthSensor.getPressure();
        pressurePub.publish(&pressure_m);
        depthSensorSchedule += DEPTH_INTERVAL;
        toggleLed();
      }
    } else {
      nh.logerror("Depth Sensor is NOT CONNECTED!!");
      depthSensorSchedule += DEPTH_DISCONNECT_INTERVAL;
      reconnectDepthSensor();
      toggleLed();
    }
  }
  nh.spinOnce();
}
