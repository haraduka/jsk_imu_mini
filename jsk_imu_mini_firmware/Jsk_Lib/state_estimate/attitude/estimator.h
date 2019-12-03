/*
******************************************************************************
* File Name          : estimator.h
* Description        : super  class for attitude estiamte algorithm
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_ESTIMATOR_H
#define __ATTITUDE_ESTIMATOR_H

#include <stdint.h>
#include <math.h>

#define DELTA_T 0.01f

class EstimatorAlgorithm
{
public:
  EstimatorAlgorithm():
    acc_bodyframe_(), acc_virtualframe_(), gyro_bodyframe_(), gyro_virtualframe_(), mag_bodyframe_(), mag_virtualframe_(),q_(),
    desire_attitude_roll_(0), desire_attitude_pitch_(0),
    abs_rel_(ABSOLUTE_COORD)
  {
    r_.identity();;
  };

  ~EstimatorAlgorithm(){}

  /* coodrinate change  */
  void coordinateUpdate(float desire_attitude_roll, float desire_attitude_pitch)
  {
    desire_attitude_roll_ = desire_attitude_roll; 
    desire_attitude_pitch_ = desire_attitude_pitch; 

    r_.from_euler(desire_attitude_roll_, desire_attitude_pitch_, 0);

  }

  void update(const Vector3f& gyro, const Vector3f& acc, const Vector3f& mag)
  {
    /* the sensor data in body frame */
    acc_bodyframe_ = acc;
    gyro_bodyframe_ = gyro;
    mag_bodyframe_ = mag;

    /* the sensor data in virtual frame */
    acc_virtualframe_ = r_* acc_bodyframe_;
    gyro_virtualframe_ = r_*  gyro_bodyframe_;
    mag_virtualframe_ = r_ * mag_bodyframe_;

    estimation();
  }

  virtual void estimation(){}; //please implementation!

  static const uint8_t ABSOLUTE_COORD = 0;
  static const uint8_t RELATIVE_COORD = 1;
  static const uint8_t X = 0;
  static const uint8_t Y = 1;
  static const uint8_t Z = 2;

  Quaternion getQuaternion(){return q_;}

  Vector3f getAngles(){return rpy_;}
  Vector3f getVels(){return gyro_virtualframe_;} // should be the virtual frame

  Vector3f getAccB(){return acc_bodyframe_;}
  Vector3f getGyroB(){return gyro_bodyframe_;}
  Vector3f getMagB(){return mag_bodyframe_;}

  Vector3f getAccV(){return acc_virtualframe_;}
  Vector3f getGyroV(){return gyro_virtualframe_;}
  Vector3f getMagV(){return mag_virtualframe_;} 

protected:
  Vector3f acc_bodyframe_, acc_virtualframe_;  
  Vector3f gyro_bodyframe_, gyro_virtualframe_;
  Vector3f mag_bodyframe_, mag_virtualframe_;

  Matrix3f r_;
  Vector3f rpy_;
  Quaternion q_; //TODO => change to AP_Math

  float desire_attitude_roll_, desire_attitude_pitch_;
  uint8_t abs_rel_;
  uint8_t update_desire_attitude_;

};

#endif
