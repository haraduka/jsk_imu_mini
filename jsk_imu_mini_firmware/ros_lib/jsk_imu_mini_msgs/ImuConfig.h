#ifndef _ROS_SERVICE_ImuConfig_h
#define _ROS_SERVICE_ImuConfig_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace jsk_imu_mini_msgs
{

static const char IMUCONFIG[] = "jsk_imu_mini_msgs/ImuConfig";

  class ImuConfigRequest : public ros::Msg
  {
    public:
      typedef uint8_t _data_type;
      _data_type data;

    ImuConfigRequest():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->data >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->data =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return IMUCONFIG; };
    const char * getMD5(){ return "7c8164229e7d2c17eb95e9231617fdee"; };

  };

  class ImuConfigResponse : public ros::Msg
  {
    public:
      typedef bool _data_type;
      _data_type data;

    ImuConfigResponse():
      data(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return IMUCONFIG; };
    const char * getMD5(){ return "8b94c1b53db61fb6aed406028ad6332a"; };

  };

  class ImuConfig {
    public:
    typedef ImuConfigRequest Request;
    typedef ImuConfigResponse Response;
  };

}
#endif
