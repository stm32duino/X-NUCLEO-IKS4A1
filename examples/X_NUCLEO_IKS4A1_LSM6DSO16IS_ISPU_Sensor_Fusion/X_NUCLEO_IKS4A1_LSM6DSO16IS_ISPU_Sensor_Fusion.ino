/**
 ******************************************************************************
 * @file   X_NUCLEO_IKS4A1_LSM6DSO16IS_ISPU_Sensor_Fusion.ino
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
//NOTE: this example isn't compatible with Arduino Uno

/*
 * You can display the quaternion values with a 3D model connecting for example to this link:
 * https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/
 */

// Includes
#include "LSM6DSO16ISSensor.h"
#include "sensor_fusion.h"

#define INT_1 A5

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

//Interrupts.
volatile int mems_event = 0;

LSM6DSO16ISSensor sensor(&DEV_I2C, LSM6DSO16IS_I2C_ADD_L);
ucf_line_ext_t *ProgramPointer;
int32_t LineCounter;
int32_t TotalNumberOfLine;
void INT1Event_cb();

union data {
  uint8_t raw_data[16];
  float_t values[4];
};

void setup()
{
  // Led.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  // Initlialize i2c.
  DEV_I2C.begin();

  // Initlialize components.
  sensor.begin();
  sensor.Enable_X();
  sensor.Enable_G();

  // Feed the program to ISPU
  ProgramPointer = (ucf_line_ext_t *)ispu_conf;
  TotalNumberOfLine = sizeof(ispu_conf) / sizeof(ucf_line_ext_t);
  SerialPort.println("LSM6DSO16IS ISPU Sensor Fusion");
  SerialPort.print("UCF Number Line=");
  SerialPort.println(TotalNumberOfLine);

  for (LineCounter = 0; LineCounter < TotalNumberOfLine; LineCounter++) {
    if (ProgramPointer[LineCounter].op == MEMS_UCF_OP_WRITE) {
      if (sensor.Write_Reg(ProgramPointer[LineCounter].address, ProgramPointer[LineCounter].data)) {
        SerialPort.print("Error loading the Program to LSM6DSO16ISSensor at line: ");
        SerialPort.println(LineCounter);
        while (1) {
          // Led blinking.
          digitalWrite(LED_BUILTIN, HIGH);
          delay(250);
          digitalWrite(LED_BUILTIN, LOW);
          delay(250);
        }
      }
    } else if (ProgramPointer[LineCounter].op == MEMS_UCF_OP_DELAY) {
      delay(ProgramPointer[LineCounter].data);
    }
  }
  SerialPort.println("Program loaded inside the LSM6DSO16IS ISPU");
  //Interrupts.
  pinMode(INT_1, INPUT);
  attachInterrupt(INT_1, INT1Event_cb, RISING);
}

void loop()
{
  union data quaternions;
  // When the quaternion for the new sample is computed and available in the output registers an interrupt is generated.
  if (mems_event) {
    LSM6DSO16IS_ISPU_Status_t ispu_status;
    mems_event = 0;
    sensor.Get_ISPU_Status(&ispu_status);
    // Check if the ISPU event is from the algo00.
    if (ispu_status.ia_ispu_0) {
      // Read quaternions and print them.
      sensor.Read_ISPU_Output(LSM6DSO16IS_ISPU_DOUT_00_L, &quaternions.raw_data[0], 16);
      SerialPort.print("Quaternion: ");
      SerialPort.print(quaternions.values[3], 4);
      SerialPort.print(", ");
      SerialPort.print(-quaternions.values[1], 4);
      SerialPort.print(", ");
      SerialPort.print(quaternions.values[0], 4);
      SerialPort.print(", ");
      SerialPort.println(quaternions.values[2], 4);
    }
  }
}

void INT1Event_cb()
{
  mems_event = 1;
}
