/*
******************************************************************************
* File Name          : attitude_estimator.h
* Description        : attitude estimator interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_ESTIMATE_H
#define __ATTITUDE_ESTIMATE_H

#include "stm32f4xx_hal.h"
#include "config.h"
/* #include "arm_math.h" */
#include <math/AP_Math.h>

/* ros */
#include <ros.h>
#include <jsk_imu_mini_msgs/Imu.h>

/* sensors */
////////////////////////////////////////
// TODO: should include the super class//
////////////////////////////////////////
#include "imu/imu_mpu9250.h"

/* estiamtor algorithm */
#include "attitude_estimator/complementary_ahrs.h"
#include "attitude_estimator/madgwick_ahrs.h"

#define COMPLEMENTARY 1
#define MADWICK 2

/* please change the algorithm type according to your application */
#define ESTIMATE_TYPE COMPLEMENTARY

class AttitudeEstimator
{
public:
  AttitudeEstimator()
  {
  }
  ~AttitudeEstimator()
  {
  }

  static const uint8_t PUB_PRESCALER = 1;
  // This is hHard coding, 1000Hz /2 = 500Hz

  static const uint8_t PUB_HZ = 2;  // 500Hz

  void init(IMU* imu, ros::NodeHandle* nh)
  {
    nh_ = nh;

    imu_pub_ = new ros::Publisher("imu", &imu_msg_);
    nh_->advertise(*imu_pub_);

    imu_ = imu;

    last_pub_time_ = HAL_GetTick();

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif
  }

  void update()
  {
    if (imu_->getUpdate())
    {
      /* attitude estimation */
      if (!imu_->getCalibrated())
        return;
      estimator_->update(imu_->getGyro(), imu_->getAcc(), imu_->getMag());

      /* send message to ros*/
      if (nh_->connected())
        publish();

      /* reset update status of imu*/
      imu_->setUpdate(false);
    }
  }

  /* send message via ros protocal */
  void publish()
  {
    uint32_t now_time = HAL_GetTick();
    if (now_time - last_pub_time_ >= PUB_INTERVAL)
    {
      last_pub_time_ = now_time;
      imu_msg_.stamp = nh_->now();
      for (int i = 0; i < 3; i++)
      {
#if 0  // virtual coord
            imu_msg_.gyro_data[i] = estimator_->getGyroV()[i];
            imu_msg_.mag_data[i] = estimator_->getMagV()[i];
            imu_msg_.acc_data[i] = estimator_->getAccV()[i];
#else  // raw data
        imu_msg_.gyro_data[i] = imu_->getGyro()[i];
        imu_msg_.mag_data[i] = imu_->getMag()[i];
        imu_msg_.acc_data[i] = imu_->getAcc()[i];
#endif
        imu_msg_.angles[i] = estimator_->getAngles()[i];
      }

      imu_pub_->publish(&imu_msg_);
    }
  }

  /* receive message via ros protocal */
  inline const Vector3f getAngles()
  {
    return estimator_->getAngles();
  }

  inline const Vector3f getVels()
  {
    return estimator_->getVels();
  }

  static const uint8_t PUB_INTERVAL = 2;

  // private:
  ros::NodeHandle* nh_;
  ros::Publisher* imu_pub_;
  jsk_imu_mini_msgs::Imu imu_msg_;

  EstimatorAlgorithm* estimator_;
  IMU* imu_;

  uint32_t last_pub_time_;

};
#endif
