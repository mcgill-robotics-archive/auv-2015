#ifndef _ROS_arduino_msgs_Motor_h
#define _ROS_arduino_msgs_Motor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arduino_msgs
{

  class Motor : public ros::Msg
  {
    public:
      int16_t Motor1;
      int16_t Motor2;
      int16_t Motor3;
      int16_t Motor4;
      int16_t Motor5;
      int16_t Motor6;
      int16_t Motor7;
      int16_t Motor8;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_Motor1;
      u_Motor1.real = this->Motor1;
      *(outbuffer + offset + 0) = (u_Motor1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor1);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor2;
      u_Motor2.real = this->Motor2;
      *(outbuffer + offset + 0) = (u_Motor2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor2);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor3;
      u_Motor3.real = this->Motor3;
      *(outbuffer + offset + 0) = (u_Motor3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor3);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor4;
      u_Motor4.real = this->Motor4;
      *(outbuffer + offset + 0) = (u_Motor4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor4);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor5;
      u_Motor5.real = this->Motor5;
      *(outbuffer + offset + 0) = (u_Motor5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor5.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor5);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor6;
      u_Motor6.real = this->Motor6;
      *(outbuffer + offset + 0) = (u_Motor6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor6.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor6);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor7;
      u_Motor7.real = this->Motor7;
      *(outbuffer + offset + 0) = (u_Motor7.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor7.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor7);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor8;
      u_Motor8.real = this->Motor8;
      *(outbuffer + offset + 0) = (u_Motor8.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Motor8.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Motor8);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_Motor1;
      u_Motor1.base = 0;
      u_Motor1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor1 = u_Motor1.real;
      offset += sizeof(this->Motor1);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor2;
      u_Motor2.base = 0;
      u_Motor2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor2 = u_Motor2.real;
      offset += sizeof(this->Motor2);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor3;
      u_Motor3.base = 0;
      u_Motor3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor3 = u_Motor3.real;
      offset += sizeof(this->Motor3);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor4;
      u_Motor4.base = 0;
      u_Motor4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor4 = u_Motor4.real;
      offset += sizeof(this->Motor4);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor5;
      u_Motor5.base = 0;
      u_Motor5.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor5.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor5 = u_Motor5.real;
      offset += sizeof(this->Motor5);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor6;
      u_Motor6.base = 0;
      u_Motor6.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor6.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor6 = u_Motor6.real;
      offset += sizeof(this->Motor6);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor7;
      u_Motor7.base = 0;
      u_Motor7.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor7.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor7 = u_Motor7.real;
      offset += sizeof(this->Motor7);
      union {
        int16_t real;
        uint16_t base;
      } u_Motor8;
      u_Motor8.base = 0;
      u_Motor8.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Motor8.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Motor8 = u_Motor8.real;
      offset += sizeof(this->Motor8);
     return offset;
    }

    const char * getType(){ return "arduino_msgs/Motor"; };
    const char * getMD5(){ return "ede24e8c7884b1a6238f81f9f1c05866"; };

  };

}
#endif