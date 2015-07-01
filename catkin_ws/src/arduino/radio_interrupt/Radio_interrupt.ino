#include "ros.h"
#include <auv_msgs/Radio.h>

ros::NodeHandle nh;

auv_msgs::Radio radio_msg;
ros::Publisher radio_pub("radio", &radio_msg);

#define pin0 0
#define pin1 1
#define pin2 2
#define pin3 3
#define pin4 4
#define pin5 5
#define pin6 6
#define pin7 7

#define TRANSMIT_ENABLE_PIN 11

unsigned long pulseStart[8];// just tmp data storage
unsigned long pulseWidth[8];


void pulseCalc(int pin) {
  if(digitalRead(pin) == HIGH) {
    pulseStart[pin] = micros();
  }
  else {
    pulseWidth[pin] = micros() - pulseStart[pin];
  }
}


//pin0 ISR
void calc0() {
  pulseCalc(pin0);
}

// pin1 ISR
void calc1() {
  pulseCalc(pin1);
}

// pin2 ISR
void calc2() {
  pulseCalc(pin2);
}

// pin3 ISR
void calc3() {
  pulseCalc(pin3);
}

// pin4 ISR
void calc4() {
  pulseCalc(pin4);
}

// pin5 ISR
void calc5() {
  pulseCalc(pin5);
}

// pin6 ISR
void calc6() {
  pulseCalc(pin6);
}

// pin7 ISR
void calc7() {
  pulseCalc(pin7);
}

//---------------------------------------------------------------


void setup() {
  
  // Attach ISRs to pins 0 - 7

  pinMode(pin0, INPUT_PULLUP);
  attachInterrupt(pin0, calc0, CHANGE);

  pinMode(pin1, INPUT_PULLUP);
  attachInterrupt(pin1, calc1, CHANGE);

  pinMode(pin2, INPUT_PULLUP);
  attachInterrupt(pin2, calc2, CHANGE);

  pinMode(pin3, INPUT_PULLUP);
  attachInterrupt(pin3, calc3, CHANGE);

  pinMode(pin4, INPUT_PULLUP);
  attachInterrupt(pin4, calc4, CHANGE);

  pinMode(pin5, INPUT_PULLUP);
  attachInterrupt(pin5, calc5, CHANGE);

  pinMode(pin6, INPUT_PULLUP);
  attachInterrupt(pin6, calc6, CHANGE);

  pinMode(pin7, INPUT_PULLUP);
  attachInterrupt(pin7, calc7, CHANGE);

  //----------------------------------------------------------------- 
  
  //ROS initialization
  nh.initNode();
  nh.advertise(radio_pub);
  
  Serial2.transmitterEnable(TRANSMIT_ENABLE_PIN);
}

void loop() {

  radio_msg.ch0 = pulseWidth[0];
  radio_msg.ch1 = pulseWidth[1];
  radio_msg.ch2 = pulseWidth[2];
  radio_msg.ch3 = pulseWidth[3];
  radio_msg.ch4 = pulseWidth[4];
  radio_msg.ch5 = pulseWidth[5];
  radio_msg.ch6 = pulseWidth[6];
  radio_msg.ch7 = pulseWidth[7];
  
  radio_pub.publish( &radio_msg );
  nh.spinOnce();
  delay(50);
}
