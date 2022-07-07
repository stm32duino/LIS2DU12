/*
   @file    LIS2DU12_DataLog_Terminal.ino
   @author  Giuseppe Roberti <giuseppe.roberti@ieee.org>
   @brief   Example to use the LIS2DU12 advanced Ultra Low Power 3D
            accelerometer
 *******************************************************************************
   Copyright (c) 2022, STMicroelectronics
   All rights reserved.
   This software component is licensed by ST under BSD 3-Clause license,
   the "License"; You may not use this file except in compliance with the
   License. You may obtain a copy of the License at:
                          opensource.org/licenses/BSD-3-Clause
 *******************************************************************************
*/

#include <LIS2DU12Sensor.h>

LIS2DU12Sensor sensor (&Wire);
uint8_t sensor_id;
int32_t acceleration[3];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  sensor.begin();
  sensor.Enable_X();
  sensor.ReadID(&sensor_id);
}

void loop() {
  sensor.Get_X_Axes(acceleration);
  Serial.printf(
    "Id:%d, X:%d, Y:%d, Z:%d\r\n",
    sensor_id, acceleration[0], acceleration[1], acceleration[2]);
  blink(LED_BUILTIN);
}

void blink(uint8_t pin) {
  digitalWrite(pin, HIGH);
  delay(25);
  digitalWrite(pin, LOW);
  delay(975);
}
