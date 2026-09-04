/*
  @file    X_NUCLEO_IKS4A1_HelloWorld_I3C.ino
   @author  STMicroelectronics
   @brief   Example to use the LIS2DUXS12 LPS22DF and LSM6DSV16X sensors with I3C and SETDASA command
 *******************************************************************************
   Copyright (c) 2026, STMicroelectronics
   All rights reserved.

   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause

 *******************************************************************************
*/

#include "LIS2DUXS12Sensor.h"
#include "LPS22DFSensor.h"
#include "LSM6DSV16XSensor.h"

#define LIS2DUXS12_DYNAMIC_ADDRESS 0x30
#define LSM6DSV16X_DYNAMIC_ADDRESS 0x33
#define LPS22DF_DYNAMIC_ADDRESS 0x36

LIS2DUXS12Sensor acc(&I3C, LIS2DUXS12_I3C_ADD_H);
LPS22DFSensor tempPress(&I3C, LPS22DF_I3C_ADD_H);
LSM6DSV16XSensor accGyro(&I3C, LSM6DSV16X_I3C_ADD_H);

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  if (!I3C.begin(I3C_SDA, I3C_SCL, 1000000U)) {
    Serial.println("begin() failed");
    while (1) {}
  }

  if (!I3C.resetDynamicAddresses()) {
    Serial.println("resetDynamicAddresses() failed");
    while (1) {}
  }
  if (!I3C.assignDynamicAddress(acc.getStaticAddress(), LIS2DUXS12_DYNAMIC_ADDRESS)) {
    Serial.println("assignDynamicAddress() failed");
    while (1) {}
  }
  if (!I3C.assignDynamicAddress(tempPress.getStaticAddress(), LPS22DF_DYNAMIC_ADDRESS)) {
    Serial.println("assignDynamicAddress() failed");
    while (1) {}
  }  
  if (!I3C.assignDynamicAddress(accGyro.getStaticAddress(), LSM6DSV16X_DYNAMIC_ADDRESS)) {
    Serial.println("assignDynamicAddress() failed");
    while (1) {}
  }
  if (acc.begin(LIS2DUXS12_DYNAMIC_ADDRESS) != LIS2DUXS12_STATUS_OK) {
    Serial.println("acc.begin() failed");
    while (1) {}
  }
  if (tempPress.begin(LPS22DF_DYNAMIC_ADDRESS) != LPS22DF_OK) {
    Serial.println("tempPress.begin() failed");
    while (1) {}
  }
  if (accGyro.begin(LSM6DSV16X_DYNAMIC_ADDRESS) != LSM6DSV16X_OK) {
    Serial.println("accGyro.begin() failed");
    while (1) {}
  }
  if (!I3C.setClock(12500000)) {
    Serial.println("setClock() failed");
    while (1) {}
  }

  if (acc.Enable_X() != LIS2DUXS12_STATUS_OK) {
    Serial.println("acc.Enable_X() failed");
    while (1) {}
  }
  if (tempPress.Enable() != LPS22DF_OK) {
    Serial.println("tempPress.Enable() failed");
    while (1) {}
  }
  if (accGyro.Enable_X() != LSM6DSV16X_OK) {
    Serial.println("accGyro.Enable_X() failed");
    while (1) {}
  }
  if (accGyro.Enable_G() != LSM6DSV16X_OK) {
    Serial.println("accGyro.Enable_G() failed");
    while (1) {}
  }

}

void loop()
{
  int32_t accel1[3] = {0};
  float pressure = 0.0f;
  float temperature = 0.0f;
  int32_t accel2[3] = {0};
  int32_t angrate[3] = {0};

  if (acc.Get_X_Axes(accel1) != LIS2DUXS12_STATUS_OK ||
      tempPress.GetPressure(&pressure) != LPS22DF_OK ||
      tempPress.GetTemperature(&temperature) != LPS22DF_OK ||
      accGyro.Get_X_Axes(accel2) != LSM6DSV16X_OK ||
      accGyro.Get_G_Axes(angrate) != LSM6DSV16X_OK) {
    Serial.println("Read failed");
    return;
  }

  Serial.print("Accel-X[mg]:");
  Serial.print(accel1[0]);
  Serial.print(",Accel-Y[mg]:");
  Serial.print(accel1[1]);
  Serial.print(",Accel-Z[mg]:");
  Serial.println(accel1[2]);

  Serial.print("Pressure[hPa]:");
  Serial.print(pressure, 2);
  Serial.print(", Temperature[C]:");
  Serial.println(temperature, 2);

  Serial.print("Accel-X[mg]:");
  Serial.print(accel2[0]);
  Serial.print(",Accel-Y[mg]:");
  Serial.print(accel2[1]);
  Serial.print(",Accel-Z[mg]:");
  Serial.println(accel2[2]);

  Serial.print("AngRate-X[mdps]:");
  Serial.print(angrate[0]);
  Serial.print(",AngRate-Y[mdps]:");
  Serial.print(angrate[1]);
  Serial.print(",AngRate-Z[mdps]:");
  Serial.println(angrate[2]);

  delay(500);
}
