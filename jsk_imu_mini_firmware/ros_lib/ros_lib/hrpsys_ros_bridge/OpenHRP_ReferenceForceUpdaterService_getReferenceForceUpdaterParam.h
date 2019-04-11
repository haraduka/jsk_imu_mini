#ifndef _ROS_SERVICE_OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParam_h
#define _ROS_SERVICE_OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "hrpsys_ros_bridge/OpenHRP_ReferenceForceUpdaterService_ReferenceForceUpdaterParam.h"

namespace hrpsys_ros_bridge
{

static const char OPENHRP_REFERENCEFORCEUPDATERSERVICE_GETREFERENCEFORCEUPDATERPARAM[] = "hrpsys_ros_bridge/OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParam";

  class OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParamRequest : public ros::Msg
  {
    public:
      const char* name;

    OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParamRequest():
      name("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
     return offset;
    }

    const char * getType(){ return OPENHRP_REFERENCEFORCEUPDATERSERVICE_GETREFERENCEFORCEUPDATERPARAM; };
    const char * getMD5(){ return "c1f3d28f1b044c871e6eff2e9fc3c667"; };

  };

  class OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParamResponse : public ros::Msg
  {
    public:
      bool operation_return;
      hrpsys_ros_bridge::OpenHRP_ReferenceForceUpdaterService_ReferenceForceUpdaterParam i_param;

    OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParamResponse():
      operation_return(0),
      i_param()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_operation_return;
      u_operation_return.real = this->operation_return;
      *(outbuffer + offset + 0) = (u_operation_return.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->operation_return);
      offset += this->i_param.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_operation_return;
      u_operation_return.base = 0;
      u_operation_return.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->operation_return = u_operation_return.real;
      offset += sizeof(this->operation_return);
      offset += this->i_param.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return OPENHRP_REFERENCEFORCEUPDATERSERVICE_GETREFERENCEFORCEUPDATERPARAM; };
    const char * getMD5(){ return "5c0c40cdbf3010daad2148b9a44a4620"; };

  };

  class OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParam {
    public:
    typedef OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParamRequest Request;
    typedef OpenHRP_ReferenceForceUpdaterService_getReferenceForceUpdaterParamResponse Response;
  };

}
#endif
