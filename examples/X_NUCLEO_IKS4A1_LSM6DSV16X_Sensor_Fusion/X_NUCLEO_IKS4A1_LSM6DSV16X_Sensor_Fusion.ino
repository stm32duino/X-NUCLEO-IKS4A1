/**
 ******************************************************************************
 * @file   X_NUCLEO_IKS4A1_LSM6DSV16X_Sensor_Fusion.ino
 * @author  SRA
 * @version V1.0.0
 * @date    October 2023
 * @brief   Arduino test application for the STMicrolectronics
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

/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

// Includes.
#include <LSM6DSV16XSensor.h>

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

#define ALGO_FREQ  120U /* Algorithm frequency 120Hz */
#define ALGO_PERIOD  (1000U / ALGO_FREQ) /* Algorithm period [ms] */
unsigned long startTime, elapsedTime;
uint8_t status = 0;
uint32_t k = 0;
uint8_t tag = 0;
float quaternions[4] = {0};

LSM6DSV16XSensor accGyr(&DEV_I2C);

void setup() {
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initialize LSM6DSV16X.
  accGyr.begin();

  // Enable Sensor Fusion
  status |= accGyr.Enable_Rotation_Vector();

  if (status != LSM6DSV16X_OK) {
    SerialPort.println("LSM6DSV16X Sensor failed to init/configure");
    while (1) {
      // Led blinking.
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
    }
  }
  SerialPort.println("LSM6DSV16X SFLP Demo");
}

void loop()
{
  uint16_t fifo_samples;
  // Get start time of loop cycle
  startTime = millis();

  // Check the number of samples inside FIFO
  if (accGyr.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK) {
    SerialPort.println("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
    while (1);
  }

  // Read the FIFO if there is one stored sample
  if (fifo_samples > 0) {
    for (int i = 0; i < fifo_samples; i++) {
      accGyr.FIFO_Get_Tag(&tag);
      if (tag == 0x13) {
        accGyr.FIFO_Get_Rotation_Vector(&quaternions[0]);

        // Print Quaternion data
        SerialPort.print("Quaternion: ");
        SerialPort.print(quaternions[3], 4);
        SerialPort.print(", ");
        SerialPort.print(quaternions[0], 4);
        SerialPort.print(", ");
        SerialPort.print(quaternions[1], 4);
        SerialPort.print(", ");
        SerialPort.println(quaternions[2], 4);

        // Compute the elapsed time within loop cycle and wait
        elapsedTime = millis() - startTime;

        if ((long)(ALGO_PERIOD - elapsedTime) > 0) {
          delay(ALGO_PERIOD - elapsedTime);
        }
      }
    }
  }
}
