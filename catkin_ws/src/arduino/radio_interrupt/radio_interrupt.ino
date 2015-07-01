#include "ros.h"
#include <auv_msgs/RadioCommands.h>

ros::NodeHandle nh;

auv_msgs::RadioCommands radio_msg;
ros::Publisher radio_pub("radio", &radio_msg);

#define pin0 16
#define pin1 17
#define pin2 18
#define pin3 19
#define pin4 20
#define pin5 21
#define pin6 22
#define pin7 23

#define MAX_BOUND  2100
#define MIN_BOUND  900

#define TRANSMIT_ENABLE_PIN 11

elapsedMillis radio_timer;
elapsedMillis overflow_warning_timer;

elapsedMicros pulse_timer [8];// just tmp data storage
uint16_t pulseWidth[8];


void pulseCalc(uint8_t index, uint8_t pin) {
  if(digitalRead(pin) == HIGH) {
    pulse_timer[index] = 0;
  }
  else {
    pulseWidth[index] = pulse_timer[index];
  }
}


//pin0 ISR
void calc0() {
  pulseCalc(0,pin0);
}

// pin1 ISR
void calc1() {
  pulseCalc(1,pin1);
}

// pin2 ISR
void calc2() {
  pulseCalc(2,pin2);
}

// pin3 ISR
void calc3() {
  pulseCalc(3,pin3);
}

// pin4 ISR
void calc4() {
  pulseCalc(4,pin4);
}

// pin5 ISR
void calc5() {
  pulseCalc(5,pin5);
}

// pin6 ISR
void calc6() {
  pulseCalc(6,pin6);
}

// pin7 ISR
void calc7() {
  pulseCalc(7,pin7);
}

//---------------------------------------------------------------


void setup() {
  
    // Attach ISRs to pins

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

uint16_t boundcheck(uint16_t value){
    if(value >  MAX_BOUND){
        return MAX_BOUND;
    } else if(value < MIN_BOUND){
        return MIN_BOUND;
    } else {
        return value;    
    }
    
}

void loop() {
    if(radio_timer > 50){
        radio_msg.channel0 = boundcheck(pulseWidth[0]);
        radio_msg.channel1 = boundcheck(pulseWidth[1]);
        radio_msg.channel2 = boundcheck(pulseWidth[2]);
        radio_msg.channel3 = boundcheck(pulseWidth[3]);
        radio_msg.channel4 = boundcheck(pulseWidth[4]);
        radio_msg.channel5 = boundcheck(pulseWidth[5]);
        radio_msg.channel6 = boundcheck(pulseWidth[6]);
        radio_msg.channel7 = boundcheck(pulseWidth[7]);
        radio_pub.publish( &radio_msg );
        radio_timer = 0;
    }  
    nh.spinOnce();
  
}
