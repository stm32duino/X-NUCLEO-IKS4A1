# X-NUCLEO-IKS4A1

The X-NUCLEO-IKS4A1 is a motion MEMS and environmental sensor expansion board for the STM32 Nucleo.
It is equipped with Arduino UNO R3 connector layout, and is designed around the LSM6DSV16X 3D accelerometer and 3D gyroscope, 
the LSM6DSO16IS 3D accelerometer and 3D gyroscope with ISPU, the LIS2DUXS12 3D accelerometer, the LIS2MDL 3D magnetometer, 
the SHT40-AD1B humidity and temperature sensor, the LPS22DF pressure and temperature sensor and the STTS22H temperature sensor.
The X-NUCLEO-IKS4A1 interfaces with the STM32 microcontroller or the Arduino boards via the I²C pin.

# Examples

There are several examples with the X-NUCLEO-IKS4A1 library.
* X_NUCLEO_IKS4A1_HelloWorld: This application provides a simple example of usage of the X-NUCLEO-IKS4A1 
Expansion Board. It shows how to display on a hyperterminal the values of all on-board MEMS and environmental sensors.
* X_NUCLEO_IKS4A1_LSM6DSO16IS_ISPU_Sensor_Fusion: This application shows how to use X-NUCLEO-IKS4A1 LSM6DSO16IS Sensor Fusion features 
for reading quaternions through ISPU and display data on a hyperterminal.
* X_NUCLEO_IKS4A1_LSM6DSV16X_6DOrientation: This application shows how to use X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer 
to find out the 6D orientation and display data on a hyperterminal.
* X_NUCLEO_IKS4A1_LSM6DSV16X_DoubleTap: This application shows how to detect the double tap event using the 
X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer.
* X_NUCLEO_IKS4A1_LSM6DSV16X_FreeFallDetection: This application shows how to detect the free fall event using the 
X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer.
* X_NUCLEO_IKS4A1_LSM6DSV16X_MLC: This application shows how to detect the activity using the MLC of 
X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer.
* X_NUCLEO_IKS4A1_LSM6DSV16X_Pedometer: This application shows how to use X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer 
to count steps.
* X_NUCLEO_IKS4A1_LSM6DSV16X_Qvar_Polling: This application shows how to use LSM6DSV16X Qvar features in polling mode. 
* X_NUCLEO_IKS4A1_LSM6DSV16X_Sensor_Fusion: This application shows how to use X-NUCLEO-IKS4A1 LSM6DSV16X Sensor Fusion features 
for reading quaternions and display data on a hyperterminal.
* X_NUCLEO_IKS4A1_LSM6DSV16X_SingleTap: This application shows how to detect the single tap event using the 
X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer.
* X_NUCLEO_IKS4A1_LSM6DSV16X_TiltDetection: This application shows how to detect the tilt event using the X-NUCLEO-IKS4A1 
LSM6DSV16X accelerometer.
* X_NUCLEO_IKS4A1_LSM6DSV16X_WakeUpDetection: This application shows how to detect the wake-up event using the 
X-NUCLEO-IKS4A1 LSM6DSV16X accelerometer.

# Dependencies

The X-NUCLEO-IKS4A1 library requires the following STM32duino libraries:

* STM32duino LSM6DSV16X: https://github.com/stm32duino/LSM6DSV16X
* STM32duino LSM6DSO16IS: https://github.com/stm32duino/LSM6DSO16IS
* STM32duino LIS2DUXS12: https://github.com/stm32duino/LIS2DUXS12
* STM32duino LIS2MDL: https://github.com/stm32duino/LIS2MDL
* STM32duino SHT40-AD1B: https://github.com/stm32duino/SHT40-AD1B
* STM32duino LPS22DF: https://github.com/stm32duino/LPS22DF
* STM32duino STTS22H: https://github.com/stm32duino/STTS22H

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-IKS4A1

The X-NUCLEO-IKS4A1 datasheet is available at  
https://www.st.com/en/ecosystems/x-nucleo-iks4a1.html
