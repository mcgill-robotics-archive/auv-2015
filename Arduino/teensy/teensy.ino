#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <auv_msgs/MotorCommands.h>
#include <auv_msgs/Solenoid.h>

//Pin definitions

//PWM MOTOR
#define MOTOR_PIN_SU_ST 9 //SeabotimotorCommandValue_1 : port_surge
#define MOTOR_PIN_SU_PO 10 //SeabotimotorCommandValue_2 : starboard_surge
#define MOTOR_PIN_SW_BO 5 //T100_1 : bow_sway
#define MOTOR_PIN_SW_ST 4 //T100_2 : stern_sway
#define MOTOR_PIN_HE_BO 3 //T100_3 : port_bow_heave
#define MOTOR_PIN_HE_ST 2 //T100_4 : starboard_bow_heave
#define MOTOR_PIN_HE_PS 1 //T100_5 : port_stern_heave
#define MOTOR_PIN_HE_SS 0 //T100_6 : starboard_stern_heave

//SOLENOID, one used pin for (Pin_33)
#define SOLENOID_PIN_D_1 12
#define SOLENOID_PIN_D_2 24
#define SOLENOID_PIN_G_1 25
#define SOLENOID_PIN_G_2 26
#define SOLENOID_PIN_T_1 27
#define SOLENOID_PIN_T_2 28

//ANALOG
#define VOLTAGE_PIN_1 A0
#define VOLTAGE_PIN_2 A1
#define DEPTH_SENSOR_PIN A2
#define GRABBER_SWTCH_PIN_1 A3
#define GRABBER_SWTCH_PIN_2 A4
#define TEMPERATURE_PIN_1 A5
#define TEMPERATURE_PIN_2 A6
#define TEMPERATURE_PIN_3 A7
#define TEMPERATURE_PIN_4 A8
#define TEMPERATURE_PIN_5 A9

//TIME INTERVAL(unit microsecond)
#define MOTOR_TIMEOUT 4000          //amount of no signal required to start to reset motors
#define TEMPERATURE_INTERVAL 1000   //amount of delay between each temperatures read
#define VOLTAGE_INTERVAL 1000       //amount of delay between each voltages read
#define DEPTH_INTERVAL 20          //amount of delay between each depth read

//THRESHOLD for motor control

#define THRESHOLD_MOTOR 50
const double VOLT_RATIO = (3.3*30.9 * 24.12) / (3.9 * 1024.0 * 23.46); //(teensy voltage * total resistance / (single resisitance * mamotorCommandValue bit))

ros::NodeHandle nh;

/*
std_msgs::Int16 depth_msg;
std_msgs::Float32 batteryVoltage1_msg;
std_msgs::Float32 batteryVoltage2_msg;
std_msgs::Float32 temperature1_msg;
std_msgs::Float32 temperature2_msg;
std_msgs::Float32 temperature3_msg;
std_msgs::Float32 temperature4_msg;
std_msgs::Float32 temperature5_msg;
*/

Servo myservo[8];

unsigned long depthSensorSchedule = 0;
unsigned long batteryVoltageSchedule = 0;
unsigned long timeLastMotorCommand = 0;
unsigned long temperatureSechedule = 0;
unsigned long lastSolenoidCommand = 0;

int lastMotorCommands[] = {0,0,0,0,0,0,0};

int boundCheck(int motorCommandValue){
  if(motorCommandValue> 500 || motorCommandValue< -500){
    char msg[70];
    String("Motor Speed out of bound: " + String(motorCommandValue) +" !").toCharArray(msg,70);
    nh.logerror(msg);
    return 0;
  }
  return motorCommandValue;
}

void motorCb( const auv_msgs::MotorCommands& msg){
  int offset = 1500;
  timeLastMotorCommand = millis();
  writeMotorCb(0, msg.port_surge);
  writeMotorCb(1, msg.starboard_surge);
  writeMotorCb(2, msg.bow_sway);
  writeMotorCb(3, msg.stern_sway);
  writeMotorCb(4, msg.port_bow_heave);
  writeMotorCb(5, msg.starboard_bow_heave);
  writeMotorCb(6, msg.port_stern_heave);
  writeMotorCb(7, msg.starboard_stern_heave);
}

void writeMotorCb (int motorNumber, int motorCommandValue)
{
  if(abs(motorCommandValue-lastMotorCommands[motorNumber]) > THRESHOLD_MOTOR)
    lastMotorCommands[motorNumber] = boundCheck(motorCommandValue);
  else if (motorCommandValue-lastMotorCommands[motorNumber] > 0)
    lastMotorCommands[motorNumber] = boundCheck(lastMotorCommands[motorNumber] + THRESHOLD_MOTOR);
  else if (motorCommandValue-lastMotorCommands[motorNumber] < 0)
    lastMotorCommands[motorNumber] = boundCheck(lastMotorCommands[motorNumber] - THRESHOLD_MOTOR); 
	
  myservo[motorNumber].writeMicroseconds(1500 + lastMotorCommands[motorNumber]);
}

void resetMotor(){
  myservo[0].writeMicroseconds(1500);
  myservo[1].writeMicroseconds(1500);
  myservo[2].writeMicroseconds(1500);
  myservo[3].writeMicroseconds(1500);
  myservo[4].writeMicroseconds(1500);
  myservo[5].writeMicroseconds(1500);
  myservo[6].writeMicroseconds(1500);
  myservo[7].writeMicroseconds(1500);
}

void resetSolenoid(){
  digitalWrite(SOLENOID_PIN_T_1,LOW);
  digitalWrite(SOLENOID_PIN_T_2,LOW);
  digitalWrite(SOLENOID_PIN_G_1,LOW);
  digitalWrite(SOLENOID_PIN_G_2,LOW);
  digitalWrite(SOLENOID_PIN_D_1,LOW);
  digitalWrite(SOLENOID_PIN_D_2,LOW);
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
  
  myservo[0].attach(MOTOR_PIN_SU_ST);
  myservo[1].attach(MOTOR_PIN_SU_PO);
  myservo[2].attach(MOTOR_PIN_SW_BO);
  myservo[3].attach(MOTOR_PIN_SW_ST);
  myservo[4].attach(MOTOR_PIN_HE_BO);
  myservo[5].attach(MOTOR_PIN_HE_ST);
  myservo[6].attach(MOTOR_PIN_HE_PS);
  myservo[7].attach(MOTOR_PIN_HE_SS);
  
  resetMotor();
  resetSolenoid();
  
  pinMode(SOLENOID_PIN_T_1,OUTPUT);
  pinMode(SOLENOID_PIN_T_2,OUTPUT);
  pinMode(SOLENOID_PIN_D_1,OUTPUT);
  pinMode(SOLENOID_PIN_D_2,OUTPUT);
  pinMode(SOLENOID_PIN_G_1,OUTPUT);
  pinMode(SOLENOID_PIN_G_2,OUTPUT);
  
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

  long currentTime = millis();
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

