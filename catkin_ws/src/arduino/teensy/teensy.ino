#include <i2c_t3.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/MotorCommands.h>
#include <auv_msgs/SolenoidCommands.h>

#include "define.h"
#include "MS5803_I2C.h"

MS5803 depthSensor(ADDRESS_HIGH);
ros::NodeHandle nh;

Servo myservo[6];


bool depthSensorConnected = false;

unsigned long depthSensorSchedule = 0;
unsigned long externalTempSchedule = 0;
unsigned long powerMonitorSchedule = 0;
unsigned long timeLastMotorCommand = 0;
unsigned long lastSolenoidCommand = 0;
unsigned long MotorStatusSchedule = 0;
unsigned long MissionSchedule = 0;

int lastMotorCommands[8] = {0,0,0,0,0,0,0,0};

std_msgs::Float32 pressure_m;
std_msgs::Float32 external_temperature_m;
std_msgs::Float32 computerVoltage_m;
std_msgs::Float32 computerCurrent_m;
std_msgs::Float32 motorVoltage_m;
std_msgs::Float32 motorCurrent_m;
std_msgs::Bool mission_m;

ros::Publisher pressurePub("~pressure", &pressure_m);  // Publish the depth topic
ros::Publisher externalTemperaturePub("~external_temperature", &external_temperature_m);
ros::Publisher computerVoltagePub("~computerVoltage", &computerVoltage_m);
ros::Publisher ComputerCurrentPub("~computerCurrent", &computerCurrent_m);
ros::Publisher motorVoltagePub("~motorVoltage", &motorVoltage_m);
ros::Publisher motorCurrentPub("~motorCurrent", &motorCurrent_m);

ros::Publisher missionPub("/mission",&mission_m);



int boundCheck(int motorCommandValue){
  if(motorCommandValue> 500 || motorCommandValue< -500){
    char msg[80];
    String("Motor command out of bound: " + String(motorCommandValue) +" !").toCharArray(msg,80);
    nh.logerror(msg);
    return 0;
  }
  return motorCommandValue;
}

void writeMotorT100 (uint8_t motorNumber, int motorCommandValue)
{
  int difference = motorCommandValue-lastMotorCommands[motorNumber];
  
  if(abs(difference) < THRESHOLD_MOTOR)
    lastMotorCommands[motorNumber] = boundCheck(motorCommandValue);
  else if (difference > 0)
    lastMotorCommands[motorNumber] = boundCheck(lastMotorCommands[motorNumber] + THRESHOLD_MOTOR);
  else
    lastMotorCommands[motorNumber] = boundCheck(lastMotorCommands[motorNumber] - THRESHOLD_MOTOR); 
  myservo[motorNumber].writeMicroseconds(MOTOR_T100_RST_VALUE + lastMotorCommands[motorNumber]);
}

void writeMotorSeabotix (uint8_t motorPin, uint8_t enablePin, int motorCommandValue)
{
  int difference = motorCommandValue-lastMotorCommands[enablePin];
  if(abs(difference) < THRESHOLD_MOTOR)
    lastMotorCommands[enablePin] = boundCheck(motorCommandValue);
  
  else if (difference > 0)
    lastMotorCommands[enablePin] = boundCheck(lastMotorCommands[enablePin] + THRESHOLD_MOTOR);
  else
    lastMotorCommands[enablePin] = boundCheck(lastMotorCommands[enablePin] - THRESHOLD_MOTOR);   
  
  if(abs(lastMotorCommands[enablePin]) > MOTOR_SEABOTICX_DEADBAND)
    digitalWrite(enablePin,HIGH);
  else 
    digitalWrite(enablePin,LOW);
 
  analogWrite(motorPin, MOTOR_SEABOTIX_RST_VALUE + lastMotorCommands[enablePin]);
}

void motorCb( const auv_msgs::MotorCommands& msg){
  if(mission_m.data){
    timeLastMotorCommand = millis();
    writeMotorT100(MOTOR_PIN_PORT_SURGE, 
                   msg.bow_sway);              
    writeMotorT100(MOTOR_PIN_STARBOARD_SURGE, 
                   msg.stern_sway);
    writeMotorT100(MOTOR_PIN_PORT_BOW_HEAVE, 
                   msg.port_bow_heave);
    writeMotorT100(MOTOR_PIN_STARBOARD_BOW_HEAVE, 
                   msg.starboard_bow_heave);               
    writeMotorT100(MOTOR_PIN_PORT_STERN_HEAVE, 
                   msg.port_stern_heave);            
    writeMotorT100(MOTOR_PIN_STARBOARD_STERN_HEAVE, 
                   msg.starboard_stern_heave);
    writeMotorSeabotix(MOTOR_PIN_STARBOARD_SWAY,
                       MOTOR_ENABLE_PIN_STARBOARD_SWAY, 
                       msg.port_surge);
    writeMotorSeabotix(MOTOR_PIN_PORT_SWAY,
                       MOTOR_ENABLE_PIN_PORT_SWAY,
                       msg.starboard_surge);
  } else {
    nh.logwarn("Motor Commands Received while Mission Off!! Commands IGNORED!!");
  }
}

void resetMotor(){
  myservo[0].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[1].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[2].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[3].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[4].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[5].writeMicroseconds(MOTOR_T100_RST_VALUE);
  analogWrite(MOTOR_PIN_STARBOARD_SWAY, MOTOR_SEABOTIX_RST_VALUE);
  analogWrite(MOTOR_PIN_PORT_SWAY, MOTOR_SEABOTIX_RST_VALUE);
  digitalWrite(MOTOR_ENABLE_PIN_STARBOARD_SWAY, LOW);
  digitalWrite(MOTOR_ENABLE_PIN_PORT_SWAY, LOW);
  for(int i = 0; i < 8; i++){
  lastMotorCommands[i] = 0;
  }
  nh.loginfo("Motors got reset!");
}

void resetSolenoid(){
  digitalWrite(SOLENOID_PIN_PORT_DROPPER, LOW);
  digitalWrite(SOLENOID_PIN_STARBOARD_DROPPER, LOW);
  digitalWrite(SOLENOID_PIN_PORT_GRABBER, LOW);
  digitalWrite(SOLENOID_PIN_STARBOARD_GRABBER, LOW);
  digitalWrite(SOLENOID_PIN_PORT_TORPEDO, LOW);
  digitalWrite(SOLENOID_PIN_STARBOARD_TORPEDO, LOW);
  digitalWrite(SOLENOID_PIN_EXTRA,LOW);
}

void solenoidCb( const auv_msgs::SolenoidCommands& msg){
  if(mission_m.data){
    lastSolenoidCommand= millis();
    digitalWrite(SOLENOID_PIN_PORT_DROPPER,msg.port_dropper);
    digitalWrite(SOLENOID_PIN_STARBOARD_DROPPER,msg.starboard_dropper);
    digitalWrite(SOLENOID_PIN_PORT_GRABBER,msg.port_grabber);
    digitalWrite(SOLENOID_PIN_STARBOARD_GRABBER,msg.starboard_grabber);
    digitalWrite(SOLENOID_PIN_PORT_TORPEDO,msg.port_torpedo);
    digitalWrite(SOLENOID_PIN_STARBOARD_TORPEDO,msg.starboard_torpedo);
    digitalWrite(SOLENOID_PIN_EXTRA,msg.extra);
  } else {
    nh.logwarn("Solenoid Commands Received while Mission Off!! Commands IGNORED!!");
  }
}

ros::Subscriber<auv_msgs::SolenoidCommands> solenoidSub("~solenoid", &solenoidCb );
ros::Subscriber<auv_msgs::MotorCommands> motorSub("~motor", &motorCb );
int test;
void setup(){
  
  //Setup for T100, normal servo control
  myservo[MOTOR_PIN_PORT_SURGE].attach(MOTOR_PIN_PORT_SURGE);
  myservo[MOTOR_PIN_STARBOARD_SURGE].attach(MOTOR_PIN_STARBOARD_SURGE);
  myservo[MOTOR_PIN_PORT_BOW_HEAVE].attach(MOTOR_PIN_PORT_BOW_HEAVE);
  myservo[MOTOR_PIN_STARBOARD_BOW_HEAVE].attach(MOTOR_PIN_STARBOARD_BOW_HEAVE);
  myservo[MOTOR_PIN_PORT_STERN_HEAVE].attach(MOTOR_PIN_PORT_STERN_HEAVE);
  myservo[MOTOR_PIN_STARBOARD_STERN_HEAVE].attach(MOTOR_PIN_STARBOARD_STERN_HEAVE);
  
  //Setup for Seabotix, PWM with highier frequency
  //Calling frequency change will affect both pin
  analogWriteFrequency(MOTOR_PIN_STARBOARD_SWAY,PWM_FREQUENCY); 
  
  // Change the resolution to 0 - 1023
  analogWriteResolution(10); 
  pinMode(MOTOR_ENABLE_PIN_STARBOARD_SWAY, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN_PORT_SWAY, OUTPUT); 

  resetMotor();
  pinMode(STATUS_PIN_FAULT, INPUT);
  pinMode(STATUS_PIN_OTW, INPUT);
  
  pinMode(SOLENOID_PIN_PORT_DROPPER, OUTPUT);
  pinMode(SOLENOID_PIN_STARBOARD_DROPPER, OUTPUT);
  pinMode(SOLENOID_PIN_PORT_GRABBER, OUTPUT);
  pinMode(SOLENOID_PIN_STARBOARD_GRABBER, OUTPUT);
  pinMode(SOLENOID_PIN_PORT_TORPEDO, OUTPUT);
  pinMode(SOLENOID_PIN_STARBOARD_TORPEDO, OUTPUT);
  pinMode(SOLENOID_PIN_EXTRA, OUTPUT);
  resetSolenoid();
  
  //Analog Setup
  pinMode(COMPUTER_VOLTAGE_PIN,INPUT);
  pinMode(COMPUTER_CURRENT_PIN,INPUT);
  pinMode(MOTOR_VOLTAGE_PIN,INPUT);
  pinMode(MOTOR_CURRENT_PIN,INPUT);
  pinMode(MISSION_PIN,INPUT);
  
  pinMode(13,OUTPUT);
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_400);
  Wire1.resetBus();
  Wire1.setDefaultTimeout(100);
  depthSensor.reset();
  Wire1.beginTransmission(MS5803_I2C_ADDR);       // slave addr
  if(Wire1.endTransmission() == 0){
    depthSensorConnected = true;
    depthSensor.begin();
  } else {
    delay(10);
  }
  digitalWrite(13,depthSensorConnected);
  //ros node initialization
  nh.initNode();

  
  //ros publisher initialization
  nh.advertise(pressurePub);        //depth sensor
  nh.advertise(externalTemperaturePub);
  nh.advertise(computerVoltagePub);     //battery level
  nh.advertise(ComputerCurrentPub);
  nh.advertise(motorVoltagePub);
  nh.advertise(motorCurrentPub);
  nh.advertise(missionPub);

  //ros subscribe initialization
  nh.subscribe(motorSub);
  nh.subscribe(solenoidSub);
  
}
  
void toggleLed(){
  //digitalWrite(LED_PIN,!digitalRead(LED_PIN));
}

void loop(){

  unsigned long currentTime = millis();
  mission_m.data = digitalRead(MISSION_PIN);
  //Depth Sensing
  if(depthSensorSchedule < currentTime){
    if(depthSensorConnected){
      depthSensor.getMeasurements(ADC_4096);
      pressure_m.data = depthSensor.getPressure();
      pressurePub.publish(&pressure_m);
      depthSensorSchedule += DEPTH_INTERVAL;
      toggleLed();
    } else {
      nh.logfatal("Depth Sensor is NOT CONNECTED!!");
      depthSensorSchedule += DEPTH_DISCONNECT_INTERVAL;
      toggleLed();
    }
  }
  
  //external Temperature
  if(externalTempSchedule < currentTime){
    if(depthSensorConnected){
      depthSensor.getMeasurements(ADC_512);
      external_temperature_m.data = depthSensor.getTemperature(CELSIUS);
      externalTemperaturePub.publish(&external_temperature_m);
      externalTempSchedule += TEMPERATURE_INTERVAL;
      toggleLed();
    } else{
      externalTempSchedule += TEMPERATURE_INTERVAL;
      toggleLed();
    }
  }
  
  if(powerMonitorSchedule < currentTime){
    computerVoltage_m.data = analogRead(COMPUTER_VOLTAGE_PIN) * kCOM_VOLT_SLOPE + kCOM_VOLT_OFFSET;
    computerCurrent_m.data = analogRead(COMPUTER_CURRENT_PIN) * kCOM_CURR_SLOPE + kCOM_CURR_OFFSET;
    motorVoltage_m.data = analogRead(MOTOR_VOLTAGE_PIN) * kMOT_VOLT_SLOPE + kMOT_VOLT_OFFSET;
    motorCurrent_m.data = analogRead(MOTOR_CURRENT_PIN) * kMOT_CURR_SLOPE + kMOT_CURR_OFFSET;
    computerVoltagePub.publish(&computerVoltage_m);
    ComputerCurrentPub.publish(&computerCurrent_m);
    motorVoltagePub.publish(&motorVoltage_m);
    motorCurrentPub.publish(&motorCurrent_m);
    powerMonitorSchedule += POWER_MONITOR_INTERVAL;
    toggleLed();
  }
  
  //Seabotix Motor status
  if(MissionSchedule < currentTime){
    missionPub.publish(&mission_m);
    MissionSchedule += MISSION_INTERVAL;
    toggleLed();
  }
  
  
  if(MotorStatusSchedule < currentTime){
    if(mission_m.data){
      if(!digitalRead(STATUS_PIN_FAULT)){
        nh.logerror("Seabotix Fault Error!");
      }
      if(!digitalRead(STATUS_PIN_OTW)){
        nh.logerror("Seabotix OTW Error!");
      }
    }
    MotorStatusSchedule += MOTOR_STATUS_INTERVAL;
    toggleLed();
  }
  
  //Motor
  if(lastSolenoidCommand + MOTOR_TIMEOUT < currentTime){
    //nh.logerror("Solenoid Command timeout!");
    resetSolenoid();
    lastSolenoidCommand = currentTime;
    toggleLed();
  }
  
  if(timeLastMotorCommand + MOTOR_TIMEOUT < currentTime){
    if(mission_m.data){
      nh.logwarn("Motor Commands timeout!");
      timeLastMotorCommand = currentTime;
      toggleLed();
    } else {
      timeLastMotorCommand = currentTime + 500;
      toggleLed();
    }
    resetMotor();
  }
  nh.spinOnce();
}

