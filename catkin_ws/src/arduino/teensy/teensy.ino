#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/MotorCommands.h>
#include <auv_msgs/SolenoidCommands.h>

#include "define.h"

ros::NodeHandle nh;

Servo myservo[6];
 
unsigned long depthSensorSchedule = 0;
unsigned long powerMonitorSchedule = 0;
unsigned long timeLastMotorCommand = 0;
unsigned long lastSolenoidCommand = 0;
unsigned long MotorStatusSchedule = 0;

int lastMotorCommands[8] = {0,0,0,0,0,0,0,0};

int boundCheck(int motorCommandValue){
  if(motorCommandValue> 500 || motorCommandValue< -500){
    char msg[80];
    String("Motor command out of bound: " + String(motorCommandValue) +" !").toCharArray(msg,80);
    nh.logerror(msg);
    return 0;
  }
  return motorCommandValue;
}

void motorCb( const auv_msgs::MotorCommands& msg){
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
                     
  nh.loginfo("Motor Command");
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
  nh.logwarn("Motors got reset!");
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
  lastSolenoidCommand= millis();
  digitalWrite(SOLENOID_PIN_PORT_DROPPER,msg.port_dropper);
  digitalWrite(SOLENOID_PIN_STARBOARD_DROPPER,msg.starboard_dropper);
  digitalWrite(SOLENOID_PIN_PORT_GRABBER,msg.port_grabber);
  digitalWrite(SOLENOID_PIN_STARBOARD_GRABBER,msg.starboard_grabber);
  digitalWrite(SOLENOID_PIN_PORT_TORPEDO,msg.port_torpedo);
  digitalWrite(SOLENOID_PIN_STARBOARD_TORPEDO,msg.starboard_torpedo);
  digitalWrite(SOLENOID_PIN_EXTRA,msg.extra);
  nh.loginfo("Solenoid Command");
}


std_msgs::Float32 depth_m;
std_msgs::Float32 computerVoltage_m;
std_msgs::Float32 computerCurrent_m;
std_msgs::Float32 motorVoltage_m;
std_msgs::Float32 motorCurrent_m;

ros::Publisher depthPub("/electrical_interface/depth", &depth_m);  // Publish the depth topic
ros::Publisher computerVoltagePub("/electrical_interface/computerVoltage", &computerVoltage_m);
ros::Publisher ComputerCurrentPub("/electrical_interface/computerCurrent", &computerCurrent_m);
ros::Publisher motorVoltagePub("/electrical_interface/motorVoltage", &motorVoltage_m);
ros::Publisher motorCurrentPub("/electrical_interface/motorCurrent", &motorCurrent_m);

ros::Subscriber<auv_msgs::SolenoidCommands> solenoidSub("/electrical_interface/solenoid", &solenoidCb );
ros::Subscriber<auv_msgs::MotorCommands> motorSub("/electrical_interface/motor", &motorCb );


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
  
  //ros node initialization
  nh.initNode();

  
  //ros publisher initialization
  nh.advertise(depthPub);        //depth sensor
  nh.advertise(computerVoltagePub);     //battery level
  nh.advertise(ComputerCurrentPub);
  nh.advertise(motorVoltagePub);
  nh.advertise(motorCurrentPub);

  //ros subscribe initialization
  nh.subscribe(motorSub);
  nh.subscribe(solenoidSub);
  
}

void loop(){

  unsigned long currentTime = millis();

  /*
  //Depth Sensing
  if(depthSensorSchedule < currentTime){
    depth_msg.data = analogRead(DEPTH_SENSOR_PIN);
    depthPub.publish(&depth_msg);
    depthSensorSchedule += DEPTH_INTERVAL;
  }

  //Power Monitoring
  if(batteryVoltageSchedule < currentTime){
    batteryVoltage1_msg.data = analogRead(VOLTAGE_PIN_1) * VOLT_RATIO;
    batteryVoltage2_msg.data = analogRead(VOLTAGE_PIN_2) * VOLT_RATIO;

    voltagePub1.publish(&batteryVoltage1_msg);
    voltagePub2.publish(&batteryVoltage2_msg);

    batteryVoltageSchedule += VOLTAGE_INTERVAL;
  }
  */
  //Seabotix Motor status
  if(MotorStatusSchedule < currentTime){
    if(!digitalRead(STATUS_PIN_FAULT)){
    nh.logerror("Seabotix Fault Error!");
    
    }
    if(!digitalRead(STATUS_PIN_OTW)){
    nh.logerror("Seabotix OTW Error!");
    }
    MotorStatusSchedule += MOTOR_STATUS_INTERVAL;
  }
  
  //Motor
  if(lastSolenoidCommand + MOTOR_TIMEOUT < currentTime){
    //nh.logerror("Solenoid Command timeout!");
    resetSolenoid();
    lastSolenoidCommand = currentTime;
  }
  
  if(timeLastMotorCommand + MOTOR_TIMEOUT < currentTime){
    nh.logerror("Motor Command timeout!");
    resetMotor();
    timeLastMotorCommand = currentTime;
  }
  nh.spinOnce();
}

