/*
 * MAX31865.h
 *
 *  Created on: May 2, 2025
 *      Author: mtjha
 */

#ifndef INC_MAX31865_H_
#define INC_MAX31865_H_

#include "main.h"
#include <stdio.h>
#include "cmsis_os.h"

/*MAX31865 Registers*/

#define CONFIG_REG			0x00
#define RTD_REG				0x01
#define HFT_REG				0x03
#define LFT_REG				0x05
#define FAULT_REG			0x07

//BITMASKS

// VBIAS (Bit 7)
#define MAX31865_CFG_VBIAS_MASK          (1U << 7)
#define MAX31865_CFG_VBIAS_OFF           (0U << 7)
#define MAX31865_CFG_VBIAS_ON            (1U << 7)

// CONVERSION MODE (Bit 6)
#define MAX31865_CFG_CONVMODE_MASK       (1U << 6)
#define MAX31865_CFG_CONVMODE_N_OFF      (0U << 6) // Normally Off
#define MAX31865_CFG_CONVMODE_AUTO       (1U << 6) // Auto conversion

// ONE-SHOT (Bit 5)
#define MAX31865_CFG_ONESHOT_MASK        (1U << 5)
#define MAX31865_CFG_ONESHOT_OFF         (0U << 5)
#define MAX31865_CFG_ONESHOT_ENABLE      (1U << 5) // Triggers one-shot conversion

// WIRE CONFIGURATION (Bit 4)
#define MAX31865_CFG_WIRE_MASK           (1U << 4)
#define MAX31865_CFG_WIRE_2_4            (0U << 4) // 2-wire or 4-wire RTD
#define MAX31865_CFG_WIRE_3              (1U << 4) // 3-wire RTD

// FAULT DETECTION CYCLE (Bits 3:2)
#define MAX31865_CFG_FAULT_DETECT_MASK   (3U << 2) // 3U = 0b11, shift by 2
#define MAX31865_CFG_FAULT_NONE          (0U << 2) // No automatic fault detection
#define MAX31865_CFG_FAULT_RUN_1         (1U << 2) // Run 1 cycle
#define MAX31865_CFG_FAULT_RUN_2         (2U << 2) // Run 2 cycles
#define MAX31865_CFG_FAULT_RUN_AUTO      (3U << 2) // Auto fault cycle (MAX31865 specific)

// FAULT STATUS CLEAR (Bit 1)
#define MAX31865_CFG_FAULT_CLEAR_MASK    (1U << 1)
#define MAX31865_CFG_FAULT_CLEAR_NO      (0U << 1) // Do not clear faults
#define MAX31865_CFG_FAULT_CLEAR_YES     (1U << 1) // Clear fault status bits

// FILTER (Bit 0)
#define MAX31865_CFG_FILTER_MASK         (1U << 0)
#define MAX31865_CFG_FILTER_60HZ         (0U << 0)
#define MAX31865_CFG_FILTER_50HZ         (1U << 0)


typedef struct{

GPIO_TypeDef		*cs_gpio_port;
uint16_t			cs_pin;
SPI_HandleTypeDef	*spi;
uint8_t				cached_config_register;

}MAX31865_t;



void MAX31865_cfg_vbias(MAX31865_t *self, uint8_t state);
void MAX31865_cfg_conversion_mode(MAX31865_t *self, uint8_t state);
void MAX31865_cfg_rtd_type(MAX31865_t *self, uint8_t state);
void MAX31865_cfg_fault_detection(MAX31865_t *self, uint8_t state);
void MAX31865_cfg_filter(MAX31865_t *self, uint8_t state);
void MAX31865_start_one_shot(MAX31865_t *self);
void MAX31865_config_hft(MAX31865_t *self);
void MAX31865_clear_fault(MAX31865_t *self);
void MAX31865_config_lft(MAX31865_t *self);



HAL_StatusTypeDef MAX31865_init(MAX31865_t *self, GPIO_TypeDef *cs_gpio_port,uint16_t cs_pin,SPI_HandleTypeDef *spi);
float MAX31865_read_temperature(MAX31865_t *self);


#endif /* INC_MAX31865_H_ */
