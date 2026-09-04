/*
  @file    X_NUCLEO_IKS4A1_HelloWorld_I3C_ENTDAA.ino
   @author  STMicroelectronics
   @brief   Example to use the LIS2DUXS12 LPS22DF and LSM6DSV16X sensors with I3C dynamic address assignment
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

LIS2DUXS12Sensor acc(&I3C);
LPS22DFSensor tempPress(&I3C);
LSM6DSV16XSensor accGyro(&I3C);
  
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

  if (!I3C.isI3CDeviceReady(LIS2DUXS12_I3C_ADD_H)) {
    Serial.println("isI3CDeviceReady() failed");
    while (1) {}
  }
  
  I3CDiscoveredDevice devices[8] = {};
  size_t found = 0;

  if (I3C.discover(devices, 8, &found)) {
    Serial.println("discover() failed");
    while (1) {}
  }
  
  uint8_t lisDynAddr = 0U;
  uint8_t lpsDynAddr = 0U;
  uint8_t lsmDynAddr = 0U;

  for (size_t i = 0; i < found; ++i) {
    Serial.println(devices[i].pid, HEX);
    if (devices[i].pid == LIS2DUXS12_I3C_PID_H) {
      lisDynAddr = devices[i].dynAddr;
      Serial.print("lisDynAddr=");
      Serial.println(lisDynAddr, HEX);
    } else if (devices[i].pid == LPS22DF_I3C_PID_H) {
      lpsDynAddr = devices[i].dynAddr;
      Serial.print("lpsDynAddr=");
      Serial.println(lpsDynAddr, HEX);
    } else if (devices[i].pid == LSM6DSV16X_I3C_PID_H) {
      lsmDynAddr = devices[i].dynAddr;
      Serial.print("lsmDynAddr=");
      Serial.println(lsmDynAddr, HEX);
    }

    if (lisDynAddr != 0U && lpsDynAddr != 0U && lsmDynAddr != 0U) {
      break; // tutti e tre i sensori trovati, non serve continuare la scansione
    }
  }

  if (lisDynAddr == 0U) {
    Serial.println("LIS2DUXS12 not found");
    while (1) {}
  }
  if (lpsDynAddr == 0U) {
    Serial.println("LPS22DF not found");
    while (1) {}
  }
  if (lsmDynAddr == 0U) {
    Serial.println("LSM6DSV16X not found");
    while (1) {}
  }
  if (!I3C.setClock(12500000)) {
    Serial.println("setClock() failed");
    while (1) {}
  }
  if (acc.begin(lisDynAddr) != LIS2DUXS12_STATUS_OK) {
    Serial.println("acc.begin() failed");
    while (1) {}
  }

  if (tempPress.begin(lpsDynAddr) != LPS22DF_OK) {
    Serial.println("tempPress.begin() failed");
    while (1) {}
  }

  if (accGyro.begin(lsmDynAddr) != LSM6DSV16X_OK) {
    Serial.println("accGyro.begin() failed");
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

