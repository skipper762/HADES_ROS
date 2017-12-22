#ifndef _ROS_hades_com_sys_control_packet_h
#define _ROS_hades_com_sys_control_packet_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace hades_com_sys
{

  class control_packet : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint8_t data[10];

    control_packet():
      header(),
      data()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 10; i++){
      *(outbuffer + offset + 0) = (this->data[i] >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 10; i++){
      this->data[i] =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data[i]);
      }
     return offset;
    }

    const char * getType(){ return "hades_com_sys/control_packet"; };
    const char * getMD5(){ return "0a75302bf0fcc625988055ac021bda65"; };

  };

}
#endif