#ifndef _ROS_hades_base_stn_audio_packet_h
#define _ROS_hades_base_stn_audio_packet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hades_base_stn
{

  class audio_packet : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _pack_size_type;
      _pack_size_type pack_size;
      int8_t data[1024];

    audio_packet():
      header(),
      pack_size(0),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_pack_size;
      u_pack_size.real = this->pack_size;
      *(outbuffer + offset + 0) = (u_pack_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pack_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pack_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pack_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pack_size);
      for( uint32_t i = 0; i < 1024; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_datai;
      u_datai.real = this->data[i];
      *(outbuffer + offset + 0) = (u_datai.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_pack_size;
      u_pack_size.base = 0;
      u_pack_size.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pack_size.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pack_size.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pack_size.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pack_size = u_pack_size.real;
      offset += sizeof(this->pack_size);
      for( uint32_t i = 0; i < 1024; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_datai;
      u_datai.base = 0;
      u_datai.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->data[i] = u_datai.real;
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "hades_base_stn/audio_packet"; };
    const char * getMD5(){ return "9539d15446a08f34ccb5e14d96c081fd"; };

  };

}
#endif