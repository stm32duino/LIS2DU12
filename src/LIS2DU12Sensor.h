/**
 ******************************************************************************
 * @file    LIS2DU12Sensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    July 2022
 * @brief   Abstract Class of a LIS2DU12 accelerometer sensor.
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

#include "Wire.h"
#include "SPI.h"
#include "lis2du12_reg.h"


/* Defines -------------------------------------------------------------------*/

#define LIS2DU12_ACC_SENSITIVITY_FOR_FS_2G   0.976f  /**< Sensitivity value for 2g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DU12_ACC_SENSITIVITY_FOR_FS_4G   1.952f  /**< Sensitivity value for 4g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DU12_ACC_SENSITIVITY_FOR_FS_8G   3.904f  /**< Sensitivity value for 8g full scale, Low-power1 mode [mg/LSB] */
#define LIS2DU12_ACC_SENSITIVITY_FOR_FS_16G  7.808f  /**< Sensitivity value for 16g full scale, Low-power1 mode [mg/LSB] */


/* Typedefs ------------------------------------------------------------------*/

typedef enum {
  LIS2DU12_OK = 0,
  LIS2DU12_ERROR = -1
} LIS2DU12StatusTypeDef;

typedef enum {
  LIS2DU12_ULTRA_LOW_POWER_DISABLE,
  LIS2DU12_ULTRA_LOW_POWER_ENABLE
} LIS2DU12_Ultra_Low_Power_t;


/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of a LIS2DU12 pressure sensor.
 */
class LIS2DU12Sensor {
  public:
    LIS2DU12Sensor(TwoWire *i2c, uint8_t address = LIS2DU12_I2C_ADD_H);
    LIS2DU12Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);
    LIS2DU12StatusTypeDef begin();
    LIS2DU12StatusTypeDef end();
    LIS2DU12StatusTypeDef Enable_X();
    LIS2DU12StatusTypeDef Disable_X();
    LIS2DU12StatusTypeDef ReadID(uint8_t *Id);
    LIS2DU12StatusTypeDef Get_X_Axes(int32_t *Acceleration);
    LIS2DU12StatusTypeDef Get_X_AxesRaw(int16_t *value);
    LIS2DU12StatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    LIS2DU12StatusTypeDef Get_X_ODR(float *Ord);
    LIS2DU12StatusTypeDef Set_X_ODR(float Ord, LIS2DU12_Ultra_Low_Power_t Power = LIS2DU12_ULTRA_LOW_POWER_DISABLE);
    LIS2DU12StatusTypeDef Get_X_FS(float *FullScale);
    LIS2DU12StatusTypeDef Set_X_FS(float FullScale);
    LIS2DU12StatusTypeDef Set_Interrupt_Latch(uint8_t Status);
    LIS2DU12StatusTypeDef Enable_DRDY_Interrupt();
    LIS2DU12StatusTypeDef Disable_DRDY_Interrupt();
    LIS2DU12StatusTypeDef Set_X_SelfTest(uint8_t Val);
    LIS2DU12StatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    LIS2DU12StatusTypeDef ReadReg(uint8_t Reg, uint8_t *Data);
    LIS2DU12StatusTypeDef WriteReg(uint8_t Reg, uint8_t Data);

    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
    LIS2DU12StatusTypeDef Set_X_ODR_When_Enabled(float Ord, LIS2DU12_Ultra_Low_Power_t Power = LIS2DU12_ULTRA_LOW_POWER_DISABLE);
    LIS2DU12StatusTypeDef Set_X_ODR_When_Disabled(float Ord, LIS2DU12_Ultra_Low_Power_t Power = LIS2DU12_ULTRA_LOW_POWER_DISABLE);

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;

    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;

    uint8_t X_enabled;
    float X_odr;
    LIS2DU12_Ultra_Low_Power_t X_ultra_low_power;

    uint8_t initialized;
    lis2du12_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LIS2DU12_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LIS2DU12_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif /* __LIS2DU12Sensor_H__ */
