/**
  ******************************************************************************
  * @file    custom_env_sensors.h
  * @author  MEMS Application Team
  * @version V6.0.0
  * @date    16-January-2019
  * @brief   This file contains definitions for the BSP Environmental Sensors interface for custom boards
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CUSTOM_ENV_SENSORS_H__
#define __CUSTOM_ENV_SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "custom_mems_conf.h"
#include "env_sensor.h"

#ifndef USE_CUSTOM_ENV_SENSOR_HTS221_0
#define USE_CUSTOM_ENV_SENSOR_HTS221_0          1
#endif

#ifndef USE_CUSTOM_ENV_SENSOR_LPS22HB_0
#define USE_CUSTOM_ENV_SENSOR_LPS22HB_0         1
#endif

#if (USE_CUSTOM_ENV_SENSOR_HTS221_0 == 1)
#include "hts221.h"
#endif

#if (USE_CUSTOM_ENV_SENSOR_LPS22HB_0 == 1)
#include "lps22hb.h"
#endif

#if (USE_CUSTOM_ENV_SENSOR_HTS221_0 == 1)
#define CUSTOM_HTS221_0 (0)
#endif

#if (USE_CUSTOM_ENV_SENSOR_LPS22HB_0 == 1)
#define CUSTOM_LPS22HB_0 (USE_CUSTOM_ENV_SENSOR_HTS221_0)
#endif

/* Environmental Sensor instance Info */
typedef struct
{
  uint8_t Temperature;
  uint8_t Pressure;
  uint8_t Humidity;
  uint8_t LowPower;
  float   HumMaxOdr;
  float   TempMaxOdr;
  float   PressMaxOdr;
} CUSTOM_ENV_SENSOR_Capabilities_t;

typedef struct
{
  uint32_t Functions;
} CUSTOM_ENV_SENSOR_Ctx_t;

#ifndef ENV_TEMPERATURE
#define ENV_TEMPERATURE      1U
#endif
#ifndef ENV_PRESSURE
#define ENV_PRESSURE         2U
#endif
#ifndef ENV_HUMIDITY
#define ENV_HUMIDITY         4U
#endif

#define CUSTOM_ENV_FUNCTIONS_NBR    3U
#define CUSTOM_ENV_INSTANCES_NBR    (USE_CUSTOM_ENV_SENSOR_HTS221_0 + USE_CUSTOM_ENV_SENSOR_LPS22HB_0)

#if (CUSTOM_ENV_INSTANCES_NBR == 0)
#error "No environmental sensor instance has been selected"
#endif

int32_t CUSTOM_ENV_SENSOR_Init(uint32_t Instance, uint32_t Functions);
int32_t CUSTOM_ENV_SENSOR_DeInit(uint32_t Instance);
int32_t CUSTOM_ENV_SENSOR_GetCapabilities(uint32_t Instance, CUSTOM_ENV_SENSOR_Capabilities_t *Capabilities);
int32_t CUSTOM_ENV_SENSOR_ReadID(uint32_t Instance, uint8_t *Id);
int32_t CUSTOM_ENV_SENSOR_Enable(uint32_t Instance, uint32_t Function);
int32_t CUSTOM_ENV_SENSOR_Disable(uint32_t Instance, uint32_t Function);
int32_t CUSTOM_ENV_SENSOR_GetOutputDataRate(uint32_t Instance, uint32_t Function, float *Odr);
int32_t CUSTOM_ENV_SENSOR_SetOutputDataRate(uint32_t Instance, uint32_t Function, float Odr);
int32_t CUSTOM_ENV_SENSOR_GetValue(uint32_t Instance, uint32_t Function, float *Value);

#ifdef __cplusplus
}
#endif

#endif /* __CUSTOM_ENV_SENSORS_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
