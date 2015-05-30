#include "SparkFun_MS5803_I2C.h"
#include <i2c_t3.h>
#include <ros.h>
#include <std_msgs/Float32.h>

//Set up the ros node and publisher
std_msgs::Float32 pressure_msg;
std_msgs::Float32 temperature_msg;
ros::Publisher pub_pressure("pressure", &pressure_msg);
//ros::Publisher pub_temperature("temperature", &temperature_msg);
ros::NodeHandle nh;

MS5803 sensor(ADDRESS_HIGH);
long publisher_timer;

void setup()
{
  Wire1.begin(I2C_MASTER, 0x00, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_200);
  sensor.reset();
  sensor.begin();  
  nh.initNode();
  nh.advertise(pub_pressure);

}

void loop()
{

  if (millis() > publisher_timer) {

  pressure_msg.data = sensor.getPressure(ADC_4096);
  temperature_msg.data=sensor.getTemperature(CELSIUS, ADC_1024);
  pub_pressure.publish(&pressure_msg);
  //pub_pressure.publish(&temperature_msg);

  publisher_timer = millis() + 1000; //publish once a second

  }

  nh.spinOnce();
}
