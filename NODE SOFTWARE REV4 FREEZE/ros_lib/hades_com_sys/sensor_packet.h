#ifndef _ROS_hades_com_sys_sensor_packet_h
#define _ROS_hades_com_sys_sensor_packet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hades_com_sys
{

  class sensor_packet : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _type_type;
      _type_type type;
      int8_t reserved[9];
      int8_t sensor_type[10];
      int32_t data[10];

    sensor_packet():
      header(),
      type(0),
      reserved(),
      sensor_type(),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.real = this->type;
      *(outbuffer + offset + 0) = (u_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      for( uint32_t i = 0; i < 9; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_reservedi;
      u_reservedi.real = this->reserved[i];
      *(outbuffer + offset + 0) = (u_reservedi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->reserved[i]);
      }
      for( uint32_t i = 0; i < 10; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_sensor_typei;
      u_sensor_typei.real = this->sensor_type[i];
      *(outbuffer + offset + 0) = (u_sensor_typei.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sensor_type[i]);
      }
      for( uint32_t i = 0; i < 10; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_datai.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_datai.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_datai.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_type;
      u_type.base = 0;
      u_type.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->type = u_type.real;
      offset += sizeof(this->type);
      for( uint32_t i = 0; i < 9; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_reservedi;
      u_reservedi.base = 0;
      u_reservedi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->reserved[i] = u_reservedi.real;
      offset += sizeof(this->reserved[i]);
      }
      for( uint32_t i = 0; i < 10; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_sensor_typei;
      u_sensor_typei.base = 0;
      u_sensor_typei.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->sensor_type[i] = u_sensor_typei.real;
      offset += sizeof(this->sensor_type[i]);
      }
      for( uint32_t i = 0; i < 10; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_datai.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "hades_com_sys/sensor_packet"; };
    const char * getMD5(){ return "7ab720f7dc4f86f21eb3b64ece7aa25b"; };

  };

}
#endif