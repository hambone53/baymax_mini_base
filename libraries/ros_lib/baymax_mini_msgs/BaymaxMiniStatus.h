#ifndef _ROS_baymax_mini_msgs_BaymaxMiniStatus_h
#define _ROS_baymax_mini_msgs_BaymaxMiniStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace baymax_mini_msgs
{

  class BaymaxMiniStatus : public ros::Msg
  {
    public:
      uint8_t BaseState;
      float FrontIRSensorReading;
      float RearIRSensorReading;
      float RightMotorVelocityCmdMS;
      float LeftMotorVelocityCmdMS;
      float xPos_m;
      float yPos_m;
      float zPos_rad;
      float xVel_ms;
      float yVel_ms;
      float zVel_rps;

    BaymaxMiniStatus():
      BaseState(0),
      FrontIRSensorReading(0),
      RearIRSensorReading(0),
      RightMotorVelocityCmdMS(0),
      LeftMotorVelocityCmdMS(0),
      xPos_m(0),
      yPos_m(0),
      zPos_rad(0),
      xVel_ms(0),
      yVel_ms(0),
      zVel_rps(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->BaseState >> (8 * 0)) & 0xFF;
      offset += sizeof(this->BaseState);
      union {
        float real;
        uint32_t base;
      } u_FrontIRSensorReading;
      u_FrontIRSensorReading.real = this->FrontIRSensorReading;
      *(outbuffer + offset + 0) = (u_FrontIRSensorReading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_FrontIRSensorReading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_FrontIRSensorReading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_FrontIRSensorReading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->FrontIRSensorReading);
      union {
        float real;
        uint32_t base;
      } u_RearIRSensorReading;
      u_RearIRSensorReading.real = this->RearIRSensorReading;
      *(outbuffer + offset + 0) = (u_RearIRSensorReading.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RearIRSensorReading.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RearIRSensorReading.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RearIRSensorReading.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RearIRSensorReading);
      union {
        float real;
        uint32_t base;
      } u_RightMotorVelocityCmdMS;
      u_RightMotorVelocityCmdMS.real = this->RightMotorVelocityCmdMS;
      *(outbuffer + offset + 0) = (u_RightMotorVelocityCmdMS.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_RightMotorVelocityCmdMS.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_RightMotorVelocityCmdMS.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_RightMotorVelocityCmdMS.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->RightMotorVelocityCmdMS);
      union {
        float real;
        uint32_t base;
      } u_LeftMotorVelocityCmdMS;
      u_LeftMotorVelocityCmdMS.real = this->LeftMotorVelocityCmdMS;
      *(outbuffer + offset + 0) = (u_LeftMotorVelocityCmdMS.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_LeftMotorVelocityCmdMS.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_LeftMotorVelocityCmdMS.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_LeftMotorVelocityCmdMS.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->LeftMotorVelocityCmdMS);
      union {
        float real;
        uint32_t base;
      } u_xPos_m;
      u_xPos_m.real = this->xPos_m;
      *(outbuffer + offset + 0) = (u_xPos_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xPos_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xPos_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xPos_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xPos_m);
      union {
        float real;
        uint32_t base;
      } u_yPos_m;
      u_yPos_m.real = this->yPos_m;
      *(outbuffer + offset + 0) = (u_yPos_m.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yPos_m.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yPos_m.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yPos_m.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yPos_m);
      union {
        float real;
        uint32_t base;
      } u_zPos_rad;
      u_zPos_rad.real = this->zPos_rad;
      *(outbuffer + offset + 0) = (u_zPos_rad.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zPos_rad.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zPos_rad.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zPos_rad.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zPos_rad);
      union {
        float real;
        uint32_t base;
      } u_xVel_ms;
      u_xVel_ms.real = this->xVel_ms;
      *(outbuffer + offset + 0) = (u_xVel_ms.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_xVel_ms.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_xVel_ms.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_xVel_ms.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->xVel_ms);
      union {
        float real;
        uint32_t base;
      } u_yVel_ms;
      u_yVel_ms.real = this->yVel_ms;
      *(outbuffer + offset + 0) = (u_yVel_ms.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yVel_ms.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yVel_ms.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yVel_ms.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yVel_ms);
      union {
        float real;
        uint32_t base;
      } u_zVel_rps;
      u_zVel_rps.real = this->zVel_rps;
      *(outbuffer + offset + 0) = (u_zVel_rps.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zVel_rps.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zVel_rps.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zVel_rps.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zVel_rps);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->BaseState =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->BaseState);
      union {
        float real;
        uint32_t base;
      } u_FrontIRSensorReading;
      u_FrontIRSensorReading.base = 0;
      u_FrontIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_FrontIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_FrontIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_FrontIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->FrontIRSensorReading = u_FrontIRSensorReading.real;
      offset += sizeof(this->FrontIRSensorReading);
      union {
        float real;
        uint32_t base;
      } u_RearIRSensorReading;
      u_RearIRSensorReading.base = 0;
      u_RearIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RearIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RearIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RearIRSensorReading.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RearIRSensorReading = u_RearIRSensorReading.real;
      offset += sizeof(this->RearIRSensorReading);
      union {
        float real;
        uint32_t base;
      } u_RightMotorVelocityCmdMS;
      u_RightMotorVelocityCmdMS.base = 0;
      u_RightMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_RightMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_RightMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_RightMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->RightMotorVelocityCmdMS = u_RightMotorVelocityCmdMS.real;
      offset += sizeof(this->RightMotorVelocityCmdMS);
      union {
        float real;
        uint32_t base;
      } u_LeftMotorVelocityCmdMS;
      u_LeftMotorVelocityCmdMS.base = 0;
      u_LeftMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_LeftMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_LeftMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_LeftMotorVelocityCmdMS.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->LeftMotorVelocityCmdMS = u_LeftMotorVelocityCmdMS.real;
      offset += sizeof(this->LeftMotorVelocityCmdMS);
      union {
        float real;
        uint32_t base;
      } u_xPos_m;
      u_xPos_m.base = 0;
      u_xPos_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xPos_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xPos_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xPos_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xPos_m = u_xPos_m.real;
      offset += sizeof(this->xPos_m);
      union {
        float real;
        uint32_t base;
      } u_yPos_m;
      u_yPos_m.base = 0;
      u_yPos_m.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yPos_m.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yPos_m.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yPos_m.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yPos_m = u_yPos_m.real;
      offset += sizeof(this->yPos_m);
      union {
        float real;
        uint32_t base;
      } u_zPos_rad;
      u_zPos_rad.base = 0;
      u_zPos_rad.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zPos_rad.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zPos_rad.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zPos_rad.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zPos_rad = u_zPos_rad.real;
      offset += sizeof(this->zPos_rad);
      union {
        float real;
        uint32_t base;
      } u_xVel_ms;
      u_xVel_ms.base = 0;
      u_xVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_xVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_xVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_xVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->xVel_ms = u_xVel_ms.real;
      offset += sizeof(this->xVel_ms);
      union {
        float real;
        uint32_t base;
      } u_yVel_ms;
      u_yVel_ms.base = 0;
      u_yVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yVel_ms.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yVel_ms = u_yVel_ms.real;
      offset += sizeof(this->yVel_ms);
      union {
        float real;
        uint32_t base;
      } u_zVel_rps;
      u_zVel_rps.base = 0;
      u_zVel_rps.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zVel_rps.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zVel_rps.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zVel_rps.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zVel_rps = u_zVel_rps.real;
      offset += sizeof(this->zVel_rps);
     return offset;
    }

    const char * getType(){ return "baymax_mini_msgs/BaymaxMiniStatus"; };
    const char * getMD5(){ return "8addb8aa3e982ff874bd9581ac833a94"; };

  };

}
#endif