# The mini jsk_imu hardware firmware repository
This circuit can publish IMU data and 4 ADC channel data.


## Depending on
* TrueSTUDIO
* STM32CubeMx

## How to use
```
// write firmware via TrueSTUDIO
cd jsk_imu_mini_msgs
catkin bt
rosrun rosserial_server serial_node _baud:=921600 _port:=/dev/ttyUSB0
```
