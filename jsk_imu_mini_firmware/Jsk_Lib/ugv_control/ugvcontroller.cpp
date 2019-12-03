/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <rosnode.h>

#define SetRMotor(duty) htim4.Instance->CCR1 = NEU_DUTY - (duty - NEU_DUTY)
#define SetLMotor(duty) htim4.Instance->CCR2 = duty
//radio controll flag
extern uint8_t radio_flag;
static float interg_linear_l,interg_linear_r;
void RosNode::Ros_Pid_controller(encoder::ENCODER_ *enc_l,
		encoder::ENCODER_ *enc_r,
		IMU *imu)
{
	//get the linear err
	float err_linear_l = aim_linear_spd - enc_l->ave_velo;
	float err_linear_r = aim_linear_spd - enc_r->ave_velo;
	float d_err_linear_l = enc_l->ave_acc;
	float d_err_linear_r = enc_r->ave_acc;
	//get the angular err
	//-x is the yaw..y is the pitch, z is the roll
	//when install the pcb at the back...
	float err_angular = aim_angular_spd - (-imu->ave_gyro.x);
	float d_err_angular = -imu->ave_gyroacc.x;

	    //chen 0710
    /** To avoid fast change of the speed so that the robot would not
     * fall down
     * */
//	float kk = 1.5;
//	err_linear_l = err_linear_l>kk?kk:err_linear_l;
//	err_linear_l = err_linear_l<-kk?-kk:err_linear_l;
//	err_linear_r = err_linear_r>kk?kk:err_linear_r;
//	err_linear_r = err_linear_r<-kk?-kk:err_linear_r;

	//test  using the cmd_vel_msg.linear.y as Kp using angular.y as Kd
	int LMotor_duty_diff = (int)(Kp_l*err_linear_l - Kd_l*d_err_linear_l);
	int RMotor_duty_diff = (int)(Kp_l*err_linear_r - Kd_l*d_err_linear_r);

	LMotor_duty += (int)(Kp_l*err_linear_l - Kd_l*d_err_linear_l);
	RMotor_duty += (int)(Kp_l*err_linear_r - Kd_l*d_err_linear_r);

	/** use adjust Kp, when the aim speed is high, like:
	 * 2 rad/s, Kp = 5,  define Kp_a  10.0
	 * 0.5 rad/s, Kp = 20;
	 * 0.1 rad/s, Kp = 100;
	 */
	float tmp_kp = abs(aim_angular_spd)>=0.1?(Kp_a/abs(aim_angular_spd))
			:100;// 100
	//turning left imu output positive..  R forward L backward..
	LMotor_duty_diff += -(int)(tmp_kp*err_angular - Kd_a*d_err_angular);
	RMotor_duty_diff += (int)(tmp_kp*err_angular - Kd_a*d_err_angular);

    LMotor_duty -= (int)(tmp_kp*err_angular - Kd_a*d_err_angular);
    RMotor_duty += (int)(tmp_kp*err_angular - Kd_a*d_err_angular);
    //chen 0728
    /**
     * add I when around the center...
     */
    interg_linear_l += err_linear_l;
    interg_linear_r += err_linear_r;
    interg_linear_l *=0.5;
    interg_linear_r *=0.5;
//    if(aim_angular_spd==0&&aim_linear_spd==0)
//    {
//    	LMotor_duty += (int)(100*interg_linear_l);
//    	RMotor_duty += (int)(100*interg_linear_r);
//    }

    //chen 0831
    /**
     * remove the neutral part around 39000 ~ 42000
     */

    const int neutral_diff = 1100;
    //left
    if(abs(LMotor_duty - NEU_DUTY)<neutral_diff&&
    		LMotor_duty!=NEU_DUTY) //around the mid
    {
    	if(LMotor_duty_diff > 0)
    		LMotor_duty = NEU_DUTY + neutral_diff;
    	else if(LMotor_duty_diff < 0)
    		LMotor_duty = NEU_DUTY - neutral_diff;
    }
    //right
    if(abs(RMotor_duty - NEU_DUTY)<neutral_diff&&
    		RMotor_duty!=NEU_DUTY) //around the mid
    {
    	if(RMotor_duty_diff > 0)
    		RMotor_duty = NEU_DUTY + neutral_diff;
    	else if(RMotor_duty_diff < 0)
    		RMotor_duty = NEU_DUTY - neutral_diff;
    }


    //chen 0529
    /** This part is to ensure that when the aim angular speed
     *  aim_angular_spd = 0, because of the tire drag, the car cant
     *  move, then the left and right duty have a big difference cause
     *  the motor to exhaust... just simple check if aim_angular_apd is 0,
     *  and the abs(duty_r-duty_l) > a certain value..
     * */
    if(aim_angular_spd==0&&abs(LMotor_duty-RMotor_duty)>3000&&aim_linear_spd==0)
    {
    	LMotor_duty = NEU_DUTY;
    	RMotor_duty = NEU_DUTY;
    }


	// next we need to control the angular....
    odom_msg.angular.x = LMotor_duty;
    odom_msg.angular.y = RMotor_duty;
	this->radioflag_msg.data = radio_flag;
	/* if automatic control */
	if(!radio_flag)
	{
		LMotor_duty = LMotor_duty>MAX_DUTY?MAX_DUTY:LMotor_duty;
		LMotor_duty = LMotor_duty<MIN_DUTY?MIN_DUTY:LMotor_duty;
		RMotor_duty = RMotor_duty>MAX_DUTY?MAX_DUTY:RMotor_duty;
		RMotor_duty = RMotor_duty<MIN_DUTY?MIN_DUTY:RMotor_duty;
		SetLMotor(LMotor_duty);
		SetRMotor(RMotor_duty);
	}
	else // or manual control, the control will not accumulate..
	{
		LMotor_duty = NEU_DUTY;
		RMotor_duty = NEU_DUTY;
	}

}
