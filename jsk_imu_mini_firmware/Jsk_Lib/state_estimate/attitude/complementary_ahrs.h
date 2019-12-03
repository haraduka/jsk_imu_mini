/*
******************************************************************************
* File Name          : complementary_ahrs.h
* Description        : ahrs estimation by complementary filter
******************************************************************************
*/
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif


#ifndef __COMPLEMENTARY_AHRS_H
#define __COMPLEMENTARY_AHRS_H

#include "state_estimate/attitude/estimator.h"

#define GYR_CMPF_FACTOR 60
#define GYR_CMPFM_FACTOR 25
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define PRESCLAER_ACC 3 // if value=1, it means same rate with gyro, for genral attitude estimation
//* the gyro integaral attitude estiamtion amplification rate
#define GYRO_AMP  1.1395f  //if value= 1, it means normal complementrary filter

#define G_MIN 72
#define G_MAX 133

class ComplementaryAHRS: public EstimatorAlgorithm
{
public:
  ComplementaryAHRS():EstimatorAlgorithm(),
                      est_g_v_(),  est_m_v_(),
                       est_g_b_(), est_m_b_()
  {
  }


private:
  Vector3f est_g_v_,  est_m_v_;
  Vector3f est_g_b_,  est_m_b_;

  /* core esitmation process, using body frame */
  virtual void estimation() 
  {
    int  valid_acc = 0;
    static int cnt = 0;

    float acc_magnitude = acc_bodyframe_ * acc_bodyframe_; //norm?
    Vector3f est_g_b_tmp = est_g_b_;
    Vector3f est_m_b_tmp = est_m_b_;
    est_g_b_ += (est_g_b_tmp % (gyro_bodyframe_  * (DELTA_T * GYRO_AMP) )); //rotation by gyro
    est_m_b_ += (est_m_b_tmp % (gyro_bodyframe_  * (DELTA_T * GYRO_AMP) )); //rotation by gyro

    if( G_MIN < acc_magnitude && acc_magnitude < G_MAX) valid_acc = 1;
    else valid_acc = 0;

     //*********************************************************************
     //** Apply complimentary filter (Gyro drift correction) 
     //** If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range 
     //**    => we neutralize the effect of accelerometers in the angle estimation. 
     //** To do that, we just skip filter, as EstV already rotated by Gyro 
     //*********************************************************************

    est_g_b_tmp = est_g_b_;
    est_m_b_tmp = est_m_b_;

    if ( valid_acc == 1 && cnt == 0)
      est_g_b_ = (est_g_b_tmp * GYR_CMPF_FACTOR + acc_bodyframe_) * INV_GYR_CMPF_FACTOR;
    est_m_b_ = (est_m_b_tmp * GYR_CMPFM_FACTOR  + mag_bodyframe_) * INV_GYR_CMPFM_FACTOR;

    est_g_v_ = r_ * est_g_b_;
    est_m_v_ = r_ * est_m_b_;

    // Attitude of the estimated vector
    float sq_g_x_sq_g_z = est_g_v_.x * est_g_v_.x + est_g_v_.z * est_g_v_.z;
    float sq_g_y_sq_g_z = est_g_v_.y * est_g_v_.y + est_g_v_.z * est_g_v_.z;
    float invG = inv_sqrt(sq_g_x_sq_g_z + est_g_v_.y * est_g_v_.y);

    rpy_.x = atan2f(est_g_v_.y , est_g_v_.z);
    rpy_.y = atan2f(-est_g_v_.x , inv_sqrt(sq_g_y_sq_g_z)* sq_g_y_sq_g_z);
    rpy_.z = atan2f( est_m_v_.z * est_g_v_.y - est_m_v_.y * est_g_v_.z, est_m_v_.x * invG * sq_g_y_sq_g_z  - (est_m_v_.y * est_g_v_.y + est_m_v_.z * est_g_v_.z) * invG * est_g_v_.x ) ;//+ MAG_DECLINIATION;
     //********************************************************************************:
     //** refrence1: https://sites.google.com/site/myimuestimationexperience/sensors/magnetometer
     //** refrence2: http://uav.xenocross.net/hdg.html
     //********************************************************************************

    q_.from_euler( rpy_.x, rpy_.y, rpy_.z );
    q_.normalize();

    /* update */
    if(valid_acc) cnt++;
    if(cnt == PRESCLAER_ACC) cnt = 0;
  }

};


#endif
