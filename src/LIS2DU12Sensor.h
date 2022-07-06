/**
 ******************************************************************************
 * @file    LIS2DU12Sensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    July 2022
 * @brief   Abstract Class of a LIS2DU12 pressure sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2022 STMicroelectronics</center></h2>
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


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LIS2DU12Sensor_H__
#define __LIS2DU12Sensor_H__


/* Includes ------------------------------------------------------------------*/


/* Defines -------------------------------------------------------------------*/

#include "Wire.h"
#include "SPI.h"


/* Typedefs ------------------------------------------------------------------*/

typedef enum {
  LIS2DU12_OK = 0,
  LIS2DU12_ERROR = -1
} LIS2DU12StatusTypeDef;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a LIS2DU12 pressure sensor.
 */
class LIS2DU12Sensor {
  public:
    LIS2DU12Sensor(TwoWire *i2c) : dev_i2c(i2c) {};
    LIS2DU12StatusTypeDef ReadID(uint8_t *Id)
    {
      *Id = LIS2DU12_ERROR;
      return LIS2DU12_ERROR;
    };

  private:
    TwoWire *dev_i2c;
};

#endif /* __LIS2DU12Sensor_H__ */
