/**
 ******************************************************************************
 * @file    X_NUCLEO_IKS4A1_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    October 2023
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-IKS4A1
 *          MEMS Inertial and Environmental sensor expansion board.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2023 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


// Includes
#include <LSM6DSV16XSensor.h>
#include <LIS2DUXS12Sensor.h>
#include <LSM6DSO16ISSensor.h>
#include <LIS2MDLSensor.h>
#include <LPS22DFSensor.h>
#include <STTS22HSensor.h>
#include <SHT40AD1BSensor.h>

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#elif defined(ARDUINO_ARCH_STM32)
#define DEV_I2C Wire
#elif defined(ARDUINO_ARCH_AVR)
#define DEV_I2C Wire
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

// Components
LSM6DSV16XSensor AccGyr(&DEV_I2C);
LSM6DSO16ISSensor AccGyr2(&DEV_I2C, LSM6DSO16IS_I2C_ADD_L);
LIS2DUXS12Sensor Acc3(&DEV_I2C);
LIS2MDLSensor Mag(&DEV_I2C);
LPS22DFSensor PressTemp(&DEV_I2C);
SHT40AD1BSensor HumTemp(&DEV_I2C);
STTS22HSensor Temp3(&DEV_I2C);

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);
  
  // Initialize I2C bus.
  DEV_I2C.begin();
  
  AccGyr.begin();
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  AccGyr2.begin();
  AccGyr2.Enable_X();
  AccGyr2.Enable_G();
  Acc3.begin();
  Acc3.Enable_X();
  Mag.begin();
  Mag.Enable();
  PressTemp.begin();
  PressTemp.Enable();
  Temp3.begin();
  Temp3.Enable();
}

void loop() {
  // Led blinking.
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);

  // Read humidity and temperature.
  float humidity = 0, temperature = 0;
  HumTemp.GetHumidity(&humidity);
  HumTemp.GetTemperature(&temperature);

  // Read pressure and temperature.
  float pressure = 0, temperature2 = 0;
  PressTemp.GetPressure(&pressure);
  PressTemp.GetTemperature(&temperature2);

  //Read temperature
  float temperature3 = 0;
  Temp3.GetTemperature(&temperature3);

  // Read accelerometer and gyroscope.
  int32_t accelerometer[3];
  int32_t gyroscope[3];
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);

  // Read accelerometer and gyroscope.
  int32_t accelerometer2[3];
  int32_t gyroscope2[3];
  AccGyr2.Get_X_Axes(accelerometer2);
  AccGyr2.Get_G_Axes(gyroscope2);

  //Read accelerometer
  int32_t accelerometer3[3];
  Acc3.Get_X_Axes(accelerometer3);

  //Read magnetometer
  int32_t magnetometer[3];
  Mag.GetAxes(magnetometer);

  // Output data.
  SerialPort.print("| Hum[%]: ");
  SerialPort.print(humidity, 2);
  SerialPort.print(" | Temp[C]: ");
  SerialPort.print(temperature, 2);
  SerialPort.println(" |");
  SerialPort.print("| Pres[hPa]: ");
  SerialPort.print(pressure, 2);
  SerialPort.print(" | Temp2[C]: ");
  SerialPort.print(temperature2, 2);
  SerialPort.println(" |");
  SerialPort.print("| Temp3[C]: ");
  SerialPort.print(temperature3, 2);
  SerialPort.println(" |");
  SerialPort.print("| Acc[mg]: ");
  SerialPort.print(accelerometer[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer[2]);
  SerialPort.print(" | Gyr[mdps]: ");
  SerialPort.print(gyroscope[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[1]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope[2]);
  SerialPort.println(" |");
  SerialPort.print("| Acc2[mg]: ");
  SerialPort.print(accelerometer2[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer2[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer2[2]);
  SerialPort.print(" | Gyr2[mdps]: ");
  SerialPort.print(gyroscope2[0]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope2[1]);
  SerialPort.print(" ");
  SerialPort.print(gyroscope2[2]);
  SerialPort.println(" |");
  SerialPort.print("| Acc3[mg]: ");
  SerialPort.print(accelerometer3[0]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer3[1]);
  SerialPort.print(" ");
  SerialPort.print(accelerometer3[2]);
  SerialPort.println(" |");
  SerialPort.print("| Mag[mGauss]: ");
  SerialPort.print(magnetometer[0]);
  SerialPort.print(" ");
  SerialPort.print(magnetometer[1]);
  SerialPort.print(" ");
  SerialPort.print(magnetometer[2]);
  SerialPort.println(" |");
  SerialPort.println("");
}
