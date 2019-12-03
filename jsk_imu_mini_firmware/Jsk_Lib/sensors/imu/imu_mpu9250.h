/*
 ******************************************************************************
 * File Name          : imu_mpu9250.h
 * Description        : IMU(MPU9250) Interface
 ******************************************************************************
 */

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx_hal.h"
#include "config.h"
#include "math/AP_Math.h"

/* ros */
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <jsk_imu_mini_msgs/Debug.h>
#include <sensors/sensor.h>
#include <stm32f4xx_hal_flash_ex.h>

#define IMU_SPI_CS_H       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define IMU_SPI_CS_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)

/* magenetometer update at about 100Hz, but the entire sensor data polling process is at 1KHz, should add prescaler for meg */
#define MAG_PRESCALER 4
/* magnetometer has bad noise bacause of the polling process, I think. So, we use a threshold filter method to remove the outlier */
#define MAG_GENERAL_THRESH 20.0f
#define MAG_OUTLIER_MAX_COUNT 500 //= 500ms, 1count = 1ms

/*
 * Register Address
 */
#define REG_ADDR_SELF_TEST_X_GYRO  0x00
#define REG_ADDR_SELF_TEST_Y_GYRO  0x01
#define REG_ADDR_SELF_TEST_Z_GYRO  0x02
#define REG_ADDR_SELF_TEST_X_ACCEL 0x0D
#define REG_ADDR_SELF_TEST_Y_ACCEL 0x0E
#define REG_ADDR_SELF_TEST_Z_ACCEL 0x0F
#define REG_ADDR_XG_OFFSET_H       0x13
#define REG_ADDR_XG_OFFSET_L       0x14
#define REG_ADDR_YG_OFFSET_H       0x15
#define REG_ADDR_YG_OFFSET_L       0x16
#define REG_ADDR_ZG_OFFSET_H       0x17
#define REG_ADDR_ZG_OFFSET_L       0x18
#define REG_ADDR_SMPLRT_DIV        0x19
#define REG_ADDR_CONFIG            0x1A
#define REG_ADDR_GYRO_CONFIG       0x1B
#define REG_ADDR_ACCEL_CONFIG      0x1C
#define REG_ADDR_ACCEL_CONFIG_2    0x1D
#define REG_ADDR_LP_ACCEL_ODR      0x1E
#define REG_ADDR_WOM_THR           0x1F
#define REG_ADDR_FIFO_EN           0x23
// 略
#define REG_ADDR_ACCEL_XOUT_H      0x3B
#define REG_ADDR_ACCEL_XOUT_L      0x3C
#define REG_ADDR_ACCEL_YOUT_H      0x3D
#define REG_ADDR_ACCEL_YOUT_L      0x3E
#define REG_ADDR_ACCEL_ZOUT_H      0x3F
#define REG_ADDR_ACCEL_ZOUT_L      0x40
#define REG_ADDR_TEMP_OUT_H        0x41
#define REG_ADDR_TEMP_OUT_L        0x42
#define REG_ADDR_GYRO_XOUT_H       0x43
#define REG_ADDR_GYRO_XOUT_L       0x44
#define REG_ADDR_GYRO_YOUT_H       0x45
#define REG_ADDR_GYRO_YOUT_L       0x46
#define REG_ADDR_GYRO_ZOUT_H       0x47
#define REG_ADDR_GYRO_ZOUT_L       0x48
// 略
#define REG_ADDR_SIGNAL_PATH_RESET 0x68
#define REG_ADDR_MOT_DETECT_CTRL   0x69
#define REG_ADDR_USER_CTRL         0x6A
#define REG_ADDR_PWR_MGMT_1        0x6B
#define REG_ADDR_PWR_MGMT_2        0x6C
#define REG_ADDR_FIFO_COUNTH       0x72
#define REG_ADDR_FIFO_COUNTL       0x73
#define REG_ADDR_R_W               0x74
#define REG_ADDR_WHO_AM_I          0x75
#define REG_ADDR_XA_OFFSET_H       0x77
#define REG_ADDR_XA_OFFSET_L       0x78
#define REG_ADDR_YA_OFFSET_H       0x7A
#define REG_ADDR_YA_OFFSET_L       0x7B
#define REG_ADDR_ZA_OFFSET_H       0x7D
#define REG_ADDR_ZA_OFFSET_L       0x7E

//for flash memory access
#define FLASH_TIMEOUT_VALUE 50000 /* 50 s */
#define CALIB_DATA_ADDRESS  0x08010000//0x08018000; //32KB sector:  cortex m7
//0x80xxxxxx is Bloc base addresse on AXIM interface(stmf7 manual P74)
// AXIM and TCIM share the same memory, but the addresses are different
//we don't need sector erase for the writing part here, please see L57 in attitude_estimate.c

#define CALIB_DATA_SECTOR FLASH_SECTOR_4
#define CALIB_DATA_LENGTH 36  //3 * 3 * 4
#define CALIB_ACC_ADDRESS 0
#define CALIB_GYRO_ADDRESS 12
#define CALIB_MAG_ADDRESS 24
#define CALIBRATING_STEP  1000
#define CALIBRATING_MAG_STEP  1200 //about 30s (0.01s * 3000; 0.001s * 30000)


typedef Vector3f Imugyro ;

class IMU : public SENSOR_<Imugyro>
{
public:
	IMU(SPI_HandleTypeDef* hspi, ros::NodeHandle* nh);
	IMU(){};
	~IMU(){}
	void init(SPI_HandleTypeDef* hspi, ros::NodeHandle* nh);
	//chen 0526 change update function
	uint16_t update(uint16_t queue_size = DEFAULT_DATA_SIZE);
	void ledOutput();

	bool getUpdate() { return update_; }
	void setUpdate(bool update) { update_ = update; }
	bool getCalibrated();

	static const uint8_t GYRO_DLPF_CFG = 0x02;//0x04 //0: 250Hz, 0.97ms; 3: 41Hz, 5.9ms(kduino); 4: 20Hz: 9.9ms
	static const uint8_t ACC_DLPF_CFG = 0x03; //0x03: 41Hz, 11.80ms
	static const uint8_t ACC_LPF_FACTOR = 42; // old: 16
	static const uint8_t GYRO_LPF_FACTOR =12; //old: 8
	static const uint8_t MAG_ADDRESS = 0x0C;
	static const uint8_t MAG_DATA_REGISTER = 0x03;

        /*
         * bit configuration
         */
        static const uint8_t X_OFFS_USR      = 0b0000000000000000; // 16 bits
        static const uint8_t Y_OFFS_USR      = 0b0000000000000000; // 16 bits
        static const uint8_t Z_OFFS_USR      = 0b0000000000000000; // 16 bits
        static const uint8_t SMPLRT_DIV      = 0b00000000;         // 8 bits
        static const uint8_t FIFO_MODE       = 0b0;                // 1 bit
        static const uint8_t EXT_SYNC_SET    = 0b000;              // 3 bits
        static const uint8_t DLPF_CFG        = 0b000;              // 3 bits
        static const uint8_t XGYRO_Cten      = 0b0;                // 1 bit
        static const uint8_t YGYRO_Cten      = 0b0;                // 1 bit
        static const uint8_t ZGYRO_Cten      = 0b0;                // 1 bit
        static const uint8_t GYRO_FS_SEL     = 0b00;               // 2 bits
        static const uint8_t Fchoice_b       = 0b00;               // 2 bits
        static const uint8_t ax_st_en        = 0b0;                // 1 bit
        static const uint8_t ay_st_en        = 0b0;                // 1 bit
        static const uint8_t az_st_en        = 0b0;                // 1 bit
        static const uint8_t ACCEL_FS_SEL    = 0b00;               // 2 bits
        static const uint8_t accel_fchoice_b = 0b0;                // 1 bit
        static const uint8_t A_DLPFCFG       = 0b000;              // 3 bits
        static const uint8_t lposc_clksel    = 0b0000;             // 4 bits
        static const uint8_t WOM_Threshold   = 0b00000000;         // 8 bits
        static const uint8_t TEMP_OUT        = 0b0;                // 1 bit
        static const uint8_t GYRO_XOUT       = 0b0;                // 1 bit
        static const uint8_t GYRO_YOUT       = 0b0;                // 1 bit
        static const uint8_t GYRO_ZOUT       = 0b0;                // 1 bit
        static const uint8_t ACCEL           = 0b0;                // 1 bit

	/* calibration cmd form ROS */
	static const uint8_t RESET_CALIB_CMD = 0x00;
	static const uint8_t MPU_ACC_GYRO_CALIB_CMD = 0x01;
	static const uint8_t MPU_MAG_CALIB_CMD = 0x02;
	static const uint8_t MPU_CALIB_LOAD_CMD = 0x03;
	static const uint8_t MPU_CALIB_SAVE_CMD = 0x04;

	Vector3f  getAcc(){return acc_;}   // return acceleration in m/s^2
	Vector3f  getGyro(){return gyro_;} // return angle velocity in 
	Vector3f  getMag(){return mag_;}   // 

        /* imu configuration functions */
        Vector3ui getIMUGyroSelftest();
        Vector3ui getIMUAccSelfTest();
        void      setIMUGyroOffset( uint16_t, uint16_t, uint16_t );
        void      setIMUGyroOffset( Vector3ui );
        Vector3ui getIMUGyroOffset();

        void      setIMUSampleRateDivider( uint8_t );
        uint8_t   getIMUSampleRateDivider();

        void      setIMUConfiguration( uint8_t );
        uint8_t   getIMUConfiguration();
        void      setIMUFIFOMode( uint8_t );
        uint8_t   getIMUFIFOMode();
        void      setIMUExternalSyncSet( uint8_t );
        uint8_t   getIMUExternalSyncSet();
        void      setIMUDLPFconfig( uint8_t );
        uint8_t   getIMUDLPFconfig();

        void      setIMUGyroscopeConfiguration( uint8_t );
        uint8_t   getIMUGyroscopeConfiguration();
        void      setIMUGyroFullScale( uint8_t );
        uint8_t   getIMUGyroFullScale();
        void      setIMUFchoice_b( uint8_t );
        uint8_t   getIMUFchoice_b( uint8_t );

        void      setIMUAccConfiguration( uint8_t );
        uint8_t   getIMUAccConfiguration();
        void      setIMUAccFullScale( uint8_t );
        uint8_t   getIMUAccFullScale();
        void      setIMUAccConfiguration2( uint8_t );
        uint8_t   getIMUAccConfiguration2();
        void      setIMUAccFchoice_b( uint8_t );
        uint8_t   getIMUAccFchoice_b();
        void      setIMUAccDLPFconfig( uint8_t );
        uint8_t   getIMUAccDLPFconfig( uint8_t );


	//chen 0526  add gyro acc vector
	std::vector<Imugyro> gyroacc_quene;
	Imugyro ave_gyro;
	Imugyro ave_gyroacc;
#define SYSFREQ 1000;

private:

	SPI_HandleTypeDef* hspi_;

	ros::NodeHandle* nh_;
        ros::Subscriber2<std_msgs::UInt8, IMU>* imu_config_sub_;
        ros::Publisher* imu_debug_pub_;
        jsk_imu_mini_msgs::Debug imu_debug_msg_;

	uint8_t acc_gyro_calib_flag_;
	uint8_t mag_calib_flag_;
	uint8_t reset_calib_flag_;

	Vector3f raw_gyro_adc_, raw_acc_adc_, raw_mag_adc_;

	Vector3f raw_acc_, raw_gyro_, raw_mag_;
	Vector3f acc_, gyro_,  mag_;

	Vector3f raw_gyro_p_;
	Vector3f raw_acc_p_;

	int calibrate_acc_;
	int calibrate_gyro_;
	int calibrate_mag_;

	Vector3f acc_offset_;
	Vector3f gyro_offset_;
	Vector3f mag_offset_;
	Vector3f mag_max_;
	Vector3f mag_min_;

	bool mag_filtering_flag_;
	uint16_t mag_outlier_counter_;

	bool update_;

	bool ahb_suspend_flag_; // to avoid the confliction between SPI1 and USART1(ros)

	bool SPI_connection_flag_;

	void gyroInit(void);
	void accInit(void);
	void magInit(void);

	void read (void);
	void process (void);

	void mpuWrite(uint8_t address, uint8_t value);
	uint8_t mpuRead(uint8_t address);

	void readCalibData(void);
	void writeCalibData(void);

    void imuConfigCallback(const std_msgs::UInt8&);

    void debugPrint(const std::string&);
};
#endif
