#ifndef _ROS_arduino_msgs_Solenoid_h
#define _ROS_arduino_msgs_Solenoid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class Solenoid : public ros::Msg
  {
    public:
      bool solenoid1;
      bool solenoid2;
      bool solenoid3;
      bool solenoid4;
      bool solenoid5;
      bool solenoid6;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_solenoid1;
      u_solenoid1.real = this->solenoid1;
      *(outbuffer + offset + 0) = (u_solenoid1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solenoid1);
      union {
        bool real;
        uint8_t base;
      } u_solenoid2;
      u_solenoid2.real = this->solenoid2;
      *(outbuffer + offset + 0) = (u_solenoid2.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solenoid2);
      union {
        bool real;
        uint8_t base;
      } u_solenoid3;
      u_solenoid3.real = this->solenoid3;
      *(outbuffer + offset + 0) = (u_solenoid3.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solenoid3);
      union {
        bool real;
        uint8_t base;
      } u_solenoid4;
      u_solenoid4.real = this->solenoid4;
      *(outbuffer + offset + 0) = (u_solenoid4.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solenoid4);
      union {
        bool real;
        uint8_t base;
      } u_solenoid5;
      u_solenoid5.real = this->solenoid5;
      *(outbuffer + offset + 0) = (u_solenoid5.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solenoid5);
      union {
        bool real;
        uint8_t base;
      } u_solenoid6;
      u_solenoid6.real = this->solenoid6;
      *(outbuffer + offset + 0) = (u_solenoid6.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->solenoid6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_solenoid1;
      u_solenoid1.base = 0;
      u_solenoid1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solenoid1 = u_solenoid1.real;
      offset += sizeof(this->solenoid1);
      union {
        bool real;
        uint8_t base;
      } u_solenoid2;
      u_solenoid2.base = 0;
      u_solenoid2.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solenoid2 = u_solenoid2.real;
      offset += sizeof(this->solenoid2);
      union {
        bool real;
        uint8_t base;
      } u_solenoid3;
      u_solenoid3.base = 0;
      u_solenoid3.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solenoid3 = u_solenoid3.real;
      offset += sizeof(this->solenoid3);
      union {
        bool real;
        uint8_t base;
      } u_solenoid4;
      u_solenoid4.base = 0;
      u_solenoid4.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solenoid4 = u_solenoid4.real;
      offset += sizeof(this->solenoid4);
      union {
        bool real;
        uint8_t base;
      } u_solenoid5;
      u_solenoid5.base = 0;
      u_solenoid5.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solenoid5 = u_solenoid5.real;
      offset += sizeof(this->solenoid5);
      union {
        bool real;
        uint8_t base;
      } u_solenoid6;
      u_solenoid6.base = 0;
      u_solenoid6.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->solenoid6 = u_solenoid6.real;
      offset += sizeof(this->solenoid6);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/Solenoid"; };
    const char * getMD5(){ return "0efcb9185f05bca53eb2775d2b56d3f1"; };

  };

}
#endif