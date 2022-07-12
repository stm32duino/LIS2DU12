/**
 ******************************************************************************
 * @file    LIS2DU12Sensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    July 2022
 * @brief   Implementation of a LIS2DU12 accelerometer sensor.
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


/* Includes ------------------------------------------------------------------*/

#include "LIS2DU12Sensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LIS2DU12Sensor::LIS2DU12Sensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = LIS2DU12_io_write;
  reg_ctx.read_reg = LIS2DU12_io_read;
  reg_ctx.handle = (void *)this;
  dev_spi = NULL;
  X_enabled = 0L;
}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LIS2DU12Sensor::LIS2DU12Sensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LIS2DU12_io_write;
  reg_ctx.read_reg = LIS2DU12_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0L;
  X_enabled = 0L;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::begin()
{
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }

  /* Disable I3C */
  if (lis2du12_bus_mode_set(&reg_ctx, LIS2DU12_I3C_DISABLE) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. Enable BDU. */
  if (lis2du12_init_set(&reg_ctx, LIS2DU12_DRV_RDY) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  /* FIFO mode selection */
  lis2du12_fifo_md_t fifo_mode = {
    .operation = lis2du12_fifo_md_t::LIS2DU12_BYPASS,
    .store     = lis2du12_fifo_md_t::LIS2DU12_16_BIT,
    .watermark = 0,
  };

  if (lis2du12_fifo_mode_set(&reg_ctx, &fifo_mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  /* Select default output data rate. */
  X_odr = 100.0f;

  /* Select default ultra low power (disabled). */
  X_ultra_low_power = LIS2DU12_ULTRA_LOW_POWER_DISABLE;

  /* Output data rate: power down, full scale: 2g */
  lis2du12_md_t mode = {
    .odr = lis2du12_md_t::LIS2DU12_OFF,
    .fs  = lis2du12_md_t::LIS2DU12_2g,
  };

  if (lis2du12_mode_set(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  initialized = 1L;

  return LIS2DU12_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::end()
{
  /* Disable the component */
  if (Disable_X() != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  /* Reset output data rate. */
  X_odr = 0.0f;

  /* Reset ultra low power to default value (disabled). */
  X_ultra_low_power = LIS2DU12_ULTRA_LOW_POWER_DISABLE;

  initialized = 0;

  return LIS2DU12_OK;
}

/**
 * @brief  Enable LIS2DU12 Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Enable_X()
{
  /* Check if the component is already enabled */
  if (X_enabled == 1U) {
    return LIS2DU12_OK;
  }

  /* Output data rate selection. */
  if (Set_X_ODR_When_Enabled(X_odr, X_ultra_low_power) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  X_enabled = 1U;

  return LIS2DU12_OK;
}

/**
 * @brief  Disable LIS2DU12 Accelerator
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Disable_X()
{
  /* Check if the component is already disabled */
  if (X_enabled == 0U) {
    return LIS2DU12_OK;
  }

  /* Output data rate selection - power down. */
  lis2du12_md_t mode;

  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  mode.odr = lis2du12_md_t::LIS2DU12_OFF;

  if (lis2du12_mode_set(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  X_enabled = 0;

  return LIS2DU12_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::ReadID(uint8_t *Id)
{
  if (lis2du12_id_get(&reg_ctx, (lis2du12_id_t *)Id) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
 * @brief  Read data from LIS2DU12 Accelerometer
 * @param  Acceleration the pointer where the accelerometer data are stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Get_X_Axes(int32_t *Acceleration)
{
  lis2du12_md_t mode;
  lis2du12_data_t data;

  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  if (lis2du12_data_get(&reg_ctx, &mode, &data) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  Acceleration[0] = (int32_t)data.xl.mg[0];
  Acceleration[1] = (int32_t)data.xl.mg[1];
  Acceleration[2] = (int32_t)data.xl.mg[2];

  return LIS2DU12_OK;
}

/**
 * @brief  Read raw data from LIS2DU12 Accelerometer
 * @param  Value the pointer where the accelerometer raw data are stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Get_X_AxesRaw(int16_t *Value)
{
  lis2du12_md_t mode;
  lis2du12_data_t data;

  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  if (lis2du12_data_get(&reg_ctx, &mode, &data) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  Value[0] = data.xl.raw[0];
  Value[1] = data.xl.raw[1];
  Value[2] = data.xl.raw[2];

  return LIS2DU12_OK;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  Sensitivity the pointer where the accelerometer sensitivity is stored
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Get_X_Sensitivity(float *Sensitivity)
{
  lis2du12_md_t mode;

  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  switch (mode.fs) {
    case lis2du12_md_t::LIS2DU12_2g:
      *Sensitivity = LIS2DU12_ACC_SENSITIVITY_FOR_FS_2G;
      break;

    case lis2du12_md_t::LIS2DU12_4g:
      *Sensitivity = LIS2DU12_ACC_SENSITIVITY_FOR_FS_4G;
      break;

    case lis2du12_md_t::LIS2DU12_8g:
      *Sensitivity = LIS2DU12_ACC_SENSITIVITY_FOR_FS_8G;
      break;

    case lis2du12_md_t::LIS2DU12_16g:
      *Sensitivity = LIS2DU12_ACC_SENSITIVITY_FOR_FS_16G;
      break;

    default:
      *Sensitivity = -1.0f;
      return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
 * @brief  Read LIS2DU12 Accelerometer output data rate
 * @param  Odr the pointer to the output data rate
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Get_X_ODR(float *Odr)
{
  lis2du12_md_t mode;

  /* Read actual output data rate from sensor. */
  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  switch (mode.odr) {
    case lis2du12_md_t::LIS2DU12_OFF:
    case lis2du12_md_t::LIS2DU12_TRIG_PIN:
    case lis2du12_md_t::LIS2DU12_TRIG_SW:
      *Odr = 0.0f;
      break;

    case lis2du12_md_t::LIS2DU12_1Hz5_ULP:
      *Odr = 1.5f;
      break;

    case lis2du12_md_t::LIS2DU12_3Hz_ULP:
      *Odr = 3.0f;
      break;

    case lis2du12_md_t::LIS2DU12_6Hz_ULP:
    case lis2du12_md_t::LIS2DU12_6Hz:
      *Odr = 6.0f;
      break;

    case lis2du12_md_t::LIS2DU12_12Hz5:
      *Odr = 12.5f;
      break;

    case lis2du12_md_t::LIS2DU12_25Hz:
      *Odr = 25.0f;
      break;

    case lis2du12_md_t::LIS2DU12_50Hz:
      *Odr = 50.0f;
      break;

    case lis2du12_md_t::LIS2DU12_100Hz:
      *Odr = 100.0f;
      break;

    case lis2du12_md_t::LIS2DU12_200Hz:
      *Odr = 200.0f;
      break;

    case lis2du12_md_t::LIS2DU12_400Hz:
      *Odr = 400.0f;
      break;

    case lis2du12_md_t::LIS2DU12_800Hz:
      *Odr = 800.0f;
      break;

    default:
      *Odr = -1.0f;
      return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
 * @brief  Set LIS2DU12 Accelerometer output data rate
 * @param  Odr the output data rate to be set
 * @param  Power the ultra-low-power mode to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Set_X_ODR(float Odr, LIS2DU12_Ultra_Low_Power_t Power)
{
  /* Check if the component is enabled */
  if (X_enabled == 1U) {
    return Set_X_ODR_When_Enabled(Odr, Power);
  } else {
    return Set_X_ODR_When_Disabled(Odr, Power);
  }
}

/**
 * @brief  Set LIS2DU12 Accelerometer output data rate when enabled
 * @param  Odr the output data rate to be set
 * @param  Power the ultra-low-power mode to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Set_X_ODR_When_Enabled(float Odr, LIS2DU12_Ultra_Low_Power_t Power)
{
  lis2du12_md_t mode;

  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  if (Power == LIS2DU12_ULTRA_LOW_POWER_ENABLE) {
    mode.odr = (Odr <= 1.5f) ? lis2du12_md_t::LIS2DU12_1Hz5_ULP
               : (Odr <= 3.0f) ? lis2du12_md_t::LIS2DU12_3Hz_ULP
               :                 lis2du12_md_t::LIS2DU12_6Hz_ULP;
  } else {
    mode.odr = (Odr <=   6.0f) ? lis2du12_md_t::LIS2DU12_6Hz
               : (Odr <=  12.5f) ? lis2du12_md_t::LIS2DU12_12Hz5
               : (Odr <=  25.0f) ? lis2du12_md_t::LIS2DU12_25Hz
               : (Odr <=  50.0f) ? lis2du12_md_t::LIS2DU12_50Hz
               : (Odr <= 100.0f) ? lis2du12_md_t::LIS2DU12_100Hz
               : (Odr <= 200.0f) ? lis2du12_md_t::LIS2DU12_200Hz
               : (Odr <= 400.0f) ? lis2du12_md_t::LIS2DU12_400Hz
               :                   lis2du12_md_t::LIS2DU12_800Hz;
  }

  if (lis2du12_mode_set(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  /* Store the current Odr value */
  X_odr = (mode.odr == lis2du12_md_t::LIS2DU12_1Hz5_ULP) ?   1.5f
          : (mode.odr == lis2du12_md_t::LIS2DU12_3Hz_ULP)  ?   3.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_6Hz_ULP)  ?   6.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_6Hz)      ?   6.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_12Hz5)    ?  12.5f
          : (mode.odr == lis2du12_md_t::LIS2DU12_25Hz)     ?  25.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_50Hz)     ?  50.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_100Hz)    ? 100.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_200Hz)    ? 200.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_400Hz)    ? 400.0f
          : (mode.odr == lis2du12_md_t::LIS2DU12_800Hz)    ? 800.0f
          :                                    -1.0f;

  if (X_odr == -1.0f) {
    return LIS2DU12_ERROR;
  }

  /* Store the current Power value */
  X_ultra_low_power = Power;

  return LIS2DU12_OK;
}

/**
 * @brief  Set LIS2DU12 Accelerometer output data rate when disabled
 * @param  Odr the output data rate to be set
 * @param  Power the ultra-low-power mode to be used
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Set_X_ODR_When_Disabled(float Odr, LIS2DU12_Ultra_Low_Power_t Power)
{
  /* Store the new Odr value */
  if (Power == LIS2DU12_ULTRA_LOW_POWER_ENABLE) {
    X_odr = (Odr <= 1.5f) ? 1.5f
            : (Odr <= 3.0f) ? 3.0f
            :                 6.0f;
  } else {
    X_odr = (Odr <=   6.0f) ?   6.0f
            : (Odr <=  12.5f) ?  12.5f
            : (Odr <=  25.0f) ?  25.0f
            : (Odr <=  50.0f) ?  50.0f
            : (Odr <= 100.0f) ? 100.0f
            : (Odr <= 200.0f) ? 200.0f
            : (Odr <= 400.0f) ? 400.0f
            :                   800.0f;
  }

  /* Store the new Power value */
  X_ultra_low_power = Power;

  return LIS2DU12_OK;
}

/**
 * @brief  Read LIS2DU12 Accelerometer full scale
 * @param  FullScale the pointer to the full scale
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Get_X_FS(float *FullScale)
{
  LIS2DU12StatusTypeDef ret = LIS2DU12_OK;
  lis2du12_md_t mode;

  /* Read actual full scale selection from sensor. */
  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  switch (mode.fs) {
    case lis2du12_md_t::LIS2DU12_2g:
      *FullScale =  2;
      break;

    case lis2du12_md_t::LIS2DU12_4g:
      *FullScale =  4;
      break;

    case lis2du12_md_t::LIS2DU12_8g:
      *FullScale =  8;
      break;

    case lis2du12_md_t::LIS2DU12_16g:
      *FullScale = 16;
      break;

    default:
      *FullScale = -1;
      ret = LIS2DU12_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set LIS2DU12 Accelerometer full scale
 * @param  FullScale the full scale to be set
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Set_X_FS(float FullScale)
{
  lis2du12_md_t mode;

  if (lis2du12_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  /* Seems like MISRA C-2012 rule 14.3a violation but only from single file statical analysis point of view because
     the parameter passed to the function is not known at the moment of analysis */
  mode.fs = (FullScale <= 2) ? lis2du12_md_t::LIS2DU12_2g
            : (FullScale <= 4) ? lis2du12_md_t::LIS2DU12_4g
            : (FullScale <= 8) ? lis2du12_md_t::LIS2DU12_8g
            :                    lis2du12_md_t::LIS2DU12_16g;

  if (lis2du12_mode_set(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
  * @brief  Set the interrupt latch
  * @param  Status value to be written
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Set_Interrupt_Latch(uint8_t Status)
{
  lis2du12_int_mode_t mode;

  if (lis2du12_interrupt_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  switch (Status) {
    case 0:
      mode.base_sig = lis2du12_int_mode_t::LIS2DU12_INT_LEVEL;
      break;

    case 1:
      mode.base_sig = lis2du12_int_mode_t::LIS2DU12_INT_PULSED;
      break;

    case 3:
      mode.base_sig = lis2du12_int_mode_t::LIS2DU12_INT_LATCHED;
      break;

    default:
      return LIS2DU12_ERROR;
  }

  if (lis2du12_interrupt_mode_set(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
  * @brief  Enable DRDY interrupt mode
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Enable_DRDY_Interrupt()
{
  lis2du12_int_mode_t mode;

  if (lis2du12_interrupt_mode_get(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  mode.drdy_latched = PROPERTY_DISABLE;

  if (lis2du12_interrupt_mode_set(&reg_ctx, &mode) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  lis2du12_pin_int2_route_t int2_route;

  if (lis2du12_pin_int2_route_get(&reg_ctx, &int2_route) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  int2_route.drdy_xl = PROPERTY_ENABLE;

  if (lis2du12_pin_int2_route_set(&reg_ctx, &int2_route) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
  * @brief  Disable DRDY interrupt mode
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Disable_DRDY_Interrupt()
{
  lis2du12_pin_int2_route_t int2_route;

  if (lis2du12_pin_int2_route_get(&reg_ctx, &int2_route) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  int2_route.drdy_xl = PROPERTY_DISABLE;

  if (lis2du12_pin_int2_route_set(&reg_ctx, &int2_route) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
  * @brief  Set self test
  * @param  Val the value of ST in reg CTRL3
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Set_X_SelfTest(uint8_t Val)
{
  lis2du12_st_t reg;

  switch (Val) {
    case 0:
      reg = LIS2DU12_ST_DISABLE;
      break;

    case 1:
      reg = LIS2DU12_ST_POSITIVE;
      break;

    case 2:
      reg = LIS2DU12_ST_NEGATIVE;
      break;

    default:
      return LIS2DU12_ERROR;
  }

  if (lis2du12_self_test_sign_set(&reg_ctx, reg) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
  * @brief  Get the LIS2DU12 ACC data ready bit value
  * @param  Status the status of data ready bit
  * @retval 0 in case of success, an error code otherwise
  */
LIS2DU12StatusTypeDef LIS2DU12Sensor::Get_X_DRDY_Status(uint8_t *Status)
{
  lis2du12_status_t device_status;

  if (lis2du12_status_get(&reg_ctx, &device_status) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  *Status = device_status.drdy_xl;

  return LIS2DU12_OK;
}

/**
 * @brief Read the data from register
 * @param Reg register address
 * @param Data register data
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::ReadReg(uint8_t Reg, uint8_t *Data)
{
  if (lis2du12_read_reg(&reg_ctx, Reg, Data, 1) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

/**
 * @brief Write the data to register
 * @param Reg register address
 * @param Data register data
 * @retval 0 in case of success, an error code otherwise
 */
LIS2DU12StatusTypeDef LIS2DU12Sensor::WriteReg(uint8_t Reg, uint8_t Data)
{
  if (lis2du12_write_reg(&reg_ctx, Reg, &Data, 1) != LIS2DU12_OK) {
    return LIS2DU12_ERROR;
  }

  return LIS2DU12_OK;
}

int32_t LIS2DU12_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LIS2DU12Sensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LIS2DU12_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LIS2DU12Sensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
