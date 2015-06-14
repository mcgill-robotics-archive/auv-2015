#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/MotorCommands.h>
#include <auv_msgs/Solenoid.h>

#include "define.h"

ros::NodeHandle nh;

Servo myservo[6];

unsigned long depthSensorSchedule = 0;
unsigned long batteryVoltageSchedule = 0;
unsigned long timeLastMotorCommand = 0;
unsigned long temperatureSechedule = 0;
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
  writeMotorT100(MOTOR_PIN_SW_BO, msg.bow_sway);
  writeMotorT100(MOTOR_PIN_SW_ST, msg.stern_sway);
  writeMotorT100(MOTOR_PIN_HE_BO, msg.port_bow_heave);
  writeMotorT100(MOTOR_PIN_HE_ST, msg.starboard_bow_heave);
  writeMotorT100(MOTOR_PIN_HE_PS, msg.port_stern_heave);
  writeMotorT100(MOTOR_PIN_HE_SS, msg.starboard_stern_heave);
  
  writeMotorSeabotix(MOTOR_PIN_SU_ST, msg.port_surge);
  writeMotorSeabotix(MOTOR_PIN_SU_PO, msg.starboard_surge);
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

void writeMotorSeabotix (uint8_t motorNumber, int motorCommandValue)
{
  int difference = motorCommandValue-lastMotorCommands[motorNumber-3];
  if(abs(difference) < THRESHOLD_MOTOR)
    lastMotorCommands[motorNumber-3] = boundCheck(motorCommandValue);
  
  else if (difference > 0)
    lastMotorCommands[motorNumber-3] = boundCheck(lastMotorCommands[motorNumber-3] + THRESHOLD_MOTOR);
  else
    lastMotorCommands[motorNumber-3] = boundCheck(lastMotorCommands[motorNumber-3] - THRESHOLD_MOTOR);   
  if(abs(lastMotorCommands[motorNumber-3]) > MOTOR_SEABOTICX_DEADBAND)
    digitalWrite(motorNumber-3,HIGH);
  else 
    digitalWrite(motorNumber-3,LOW);
 
  analogWrite(motorNumber, MOTOR_SEABOTIX_RST_VALUE + lastMotorCommands[motorNumber-3]);
}

void resetMotor(){
  myservo[0].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[1].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[2].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[3].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[4].writeMicroseconds(MOTOR_T100_RST_VALUE);
  myservo[5].writeMicroseconds(MOTOR_T100_RST_VALUE);
  analogWrite(MOTOR_PIN_SU_ST, MOTOR_SEABOTIX_RST_VALUE);
  analogWrite(MOTOR_PIN_SU_PO, MOTOR_SEABOTIX_RST_VALUE);
  digitalWrite(MOTOR_EN_PIN_SU_ST, LOW);
  digitalWrite(MOTOR_EN_PIN_SU_PO, LOW);
  for(int i = 0; i < 8; i++){
  lastMotorCommands[i] = 0;
  }
}

void resetSolenoid(){
  digitalWrite(SOLENOID_PIN_T_1, LOW);
  digitalWrite(SOLENOID_PIN_T_2, LOW);
  digitalWrite(SOLENOID_PIN_G_1, LOW);
  digitalWrite(SOLENOID_PIN_G_2, LOW);
  digitalWrite(SOLENOID_PIN_D_1, LOW);
  digitalWrite(SOLENOID_PIN_D_2, LOW);
}

void solenoidCb( const auv_msgs::Solenoid& msg){
  digitalWrite(SOLENOID_PIN_T_1,msg.solenoid1);
  digitalWrite(SOLENOID_PIN_T_2,msg.solenoid2);
  digitalWrite(SOLENOID_PIN_G_1,msg.solenoid3);
  digitalWrite(SOLENOID_PIN_G_2,msg.solenoid4);
  digitalWrite(SOLENOID_PIN_D_1,msg.solenoid5);
  digitalWrite(SOLENOID_PIN_D_2,msg.solenoid6);
  lastSolenoidCommand= millis();
}

/*
ros::Publisher depthPub("/electrical_interface/depth", &depth_msg);  // Publish the depth topic

ros::Publisher voltagePub1("/electrical_interface/batteryVoltage1", &batteryVoltage1_msg);
ros::Publisher voltagePub2("/electrical_interface/batteryVoltage2", &batteryVoltage2_msg);

*/

/*
ros::Publisher temperaturePub1("/electrical_interface/temperature1", &temperature1_msg);
ros::Publisher temperaturePub2("/electrical_interface/temperature2", &temperature2_msg);
ros::Publisher temperaturePub3("/electrical_interface/temperature3", &temperature3_msg);
ros::Publisher temperaturePub4("/electrical_interface/temperature4", &temperature4_msg);
ros::Publisher temperaturePub5("/electrical_interface/temperature5", &temperature5_msg);
*/

ros::Subscriber<auv_msgs::Solenoid> solenoidSub("/electrical_interface/solenoid", &solenoidCb );
ros::Subscriber<auv_msgs::MotorCommands> motorSub("/electrical_interface/motor", &motorCb );


void setup(){
  
  //Setup for T100, normal servo control
  myservo[0].attach(MOTOR_PIN_SW_BO);
  myservo[1].attach(MOTOR_PIN_SW_ST);
  myservo[2].attach(MOTOR_PIN_HE_BO);
  myservo[3].attach(MOTOR_PIN_HE_ST);
  myservo[4].attach(MOTOR_PIN_HE_PS);
  myservo[5].attach(MOTOR_PIN_HE_SS);
  
  //Setup for Seabotix, PWM with highier frequency
  //Calling frequency change will affect both pin
  analogWriteFrequency(MOTOR_PIN_SU_ST,PWM_FREQUENCY); 
  
  // Change the resolution to 0 - 1023
  analogWriteResolution(10); 
  pinMode(MOTOR_EN_PIN_SU_ST,OUTPUT);
  pinMode(MOTOR_EN_PIN_SU_PO,OUTPUT); 

  resetMotor();
  pinMode(STATUS_PIN_FAULT,INPUT);
  pinMode(STATUS_PIN_OTW,INPUT);
  
  pinMode(SOLENOID_PIN_T_1,OUTPUT);
  pinMode(SOLENOID_PIN_T_2,OUTPUT);
  pinMode(SOLENOID_PIN_D_1,OUTPUT);
  pinMode(SOLENOID_PIN_D_2,OUTPUT);
  pinMode(SOLENOID_PIN_G_1,OUTPUT);
  pinMode(SOLENOID_PIN_G_2,OUTPUT);
  resetSolenoid();
  
  //ros node initialization
  nh.initNode();

  /*
  //ros publisher initialization
  nh.advertise(depthPub);        //depth sensor
  nh.advertise(voltagePub1);     //battery level
  nh.advertise(voltagePub2);
  nh.advertise(temperaturePub1);
  nh.advertise(temperaturePub2);
  nh.advertise(temperaturePub3);
  nh.advertise(temperaturePub4);
  nh.advertise(temperaturePub5);
  */
  
  //ros subscribe initialization
  nh.subscribe(motorSub);
  nh.subscribe(solenoidSub);
  
  //resetMotor();
}

void loop(){

  unsigned long currentTime = millis();
  /*
  //temperature sensing
  if(temperatureSechedule < currentTime){
   temperature1_msg.data = analogRead(TEMPERATURE_PIN_1) * TEMP_RATIO -50;
   temperaturePub1.publish(&temperature1_msg);
   temperature2_msg.data = analogRead(TEMPERATURE_PIN_2) * TEMP_RATIO -50;
   temperaturePub2.publish(&temperature2_msg);
   temperature3_msg.data = analogRead(TEMPERATURE_PIN_3) * TEMP_RATIO -50;
   temperaturePub3.publish(&temperature3_msg);
   temperature4_msg.data = analogRead(TEMPERATURE_PIN_4) * TEMP_RATIO -50;
   temperaturePub4.publish(&temperature4_msg);
   temperature5_msg.data = analogRead(TEMPERATURE_PIN_5) * TEMP_RATIO -50;
   temperaturePub5.publish(&temperature5_msg);
   temperatureSechedule += TEMPERATURE_INTERVAL;
  }

  //depth sensing
  if(depthSensorSchedule < currentTime){
    depth_msg.data = analogRead(DEPTH_SENSOR_PIN);
    depthPub.publish(&depth_msg);
    depthSensorSchedule += DEPTH_INTERVAL;
  }

  //voltages sensing
  if(batteryVoltageSchedule < currentTime){
    batteryVoltage1_msg.data = analogRead(VOLTAGE_PIN_1) * VOLT_RATIO;
    batteryVoltage2_msg.data = analogRead(VOLTAGE_PIN_2) * VOLT_RATIO;

    voltagePub1.publish(&batteryVoltage1_msg);
    voltagePub2.publish(&batteryVoltage2_msg);

    batteryVoltageSchedule += VOLTAGE_INTERVAL;
  }

  */
  if(MotorStatusSchedule < currentTime){
    if(digitalRead(STATUS_PIN_FAULT)){
    nh.logerror("Seabotix Fault Error!");
    
    }
    if(digitalRead(STATUS_PIN_OTW)){
    nh.logerror("Seabotix OTW Error!");
    }
    MotorStatusSchedule += MOTOR_STATUS_INTERVAL;
  }
  
  if(lastSolenoidCommand + MOTOR_TIMEOUT < currentTime){
    resetSolenoid();
    lastSolenoidCommand = currentTime;
  }
  
  if(timeLastMotorCommand + MOTOR_TIMEOUT < currentTime){
    resetMotor();
    timeLastMotorCommand = currentTime;
  }
  nh.spinOnce();
}

