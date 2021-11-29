# The mini jsk_imu hardware firmware repository
This circuit can publish IMU data and 4 ADC channel data.


## Depending on
* TrueSTUDIO
* STM32CubeMx

## How to write to jsk_imu_mini board
* on TrueSTUDIO, *file* -> *Open Projects from File System* and select directory with `.project` file (`jsk_imu_mini/jsk_imu_mini_firmware/firmware/TrueSTUDIO/jsk_imu_mini`)
* compile program with *project* -> *Rebuild Project*
* open file `STM32F410CB_FLASH.ld`, connect jsk_imu_mini board via ST-LINK, and *run* -> *Debug*

## How to use
```
roscd jsk_imu_mini_msgs
catkin bt
rosrun rosserial_server serial_node _baud:=921600 _port:=/dev/ttyUSB0
# if messages are received slowly / inconsistently, try next command
setserial /dev/ttyUSB0 low_latency
```
