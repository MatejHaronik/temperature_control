/*
 * MAX31865.c
 *
 *  Created on: May 2, 2025
 *      Author: mtjha
 */
#include "sensor_interface.h"
#include "MAX31865.h"
#include <math.h>
#include <string.h>
#include <stdio.h>



HAL_StatusTypeDef MAX31865_read_register(MAX31865_t *self, uint8_t register_address, uint8_t register_bytes){

	uint8_t recieved_data_buffer[2] = {0};
	uint8_t dummy_byte = 0xFF;
	register_address &= 0x7F;

	HAL_GPIO_WritePin(self->cs_gpio_port, self->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(self->spi, &register_address, 1, HAL_MAX_DELAY);

	for(int i = 0; i < register_bytes; i++){
		HAL_SPI_TransmitReceive(self->spi, &dummy_byte, &recieved_data_buffer[i], 1, HAL_MAX_DELAY);
	}

	HAL_GPIO_WritePin(self->cs_gpio_port, self->cs_pin, GPIO_PIN_SET);

	return ((uint16_t)recieved_data_buffer[0] << 8) | recieved_data_buffer[1];

}

HAL_StatusTypeDef MAX31865_write_register(MAX31865_t *self, uint8_t register_address, uint8_t *data){

	HAL_GPIO_WritePin(self->cs_gpio_port, self->cs_pin, GPIO_PIN_RESET);
	register_address |= 0x80;
	HAL_SPI_Transmit(self->spi, &register_address, 1, 100);
	HAL_SPI_Transmit(self->spi, data, 1, 100);

	if (register_address == (CONFIG_REG | 0x80)){
		self->cached_config_register = *data ;
	}

	HAL_GPIO_WritePin(self->cs_gpio_port, self->cs_pin, GPIO_PIN_SET);
}


void MAX31865_cfg_apply(MAX31865_t *self,uint8_t bit_mask, uint8_t bit_value){

	uint8_t current_config = self->cached_config_register;

	current_config = (current_config & ~bit_mask) | (bit_value & bit_mask);

	MAX31865_write_register(self, CONFIG_REG, &current_config);

}

void MAX31865_cfg_vbias(MAX31865_t *self, uint8_t state){

	uint8_t current_cached_config = self->cached_config_register;

	uint8_t current_vbias = current_cached_config & MAX31865_CFG_VBIAS_MASK;

	if (current_vbias == state) {
		printf("Already set");
	}
	else{

		MAX31865_cfg_apply(self,MAX31865_CFG_VBIAS_MASK,state);
	}
}

void MAX31865_cfg_conversion_mode(MAX31865_t *self, uint8_t state){

	uint8_t current_cached_config = self->cached_config_register;

	uint8_t current_conv_md = current_cached_config & MAX31865_CFG_CONVMODE_MASK;

	if (current_conv_md == state) {
		printf("Already set");
	}
	else{

		MAX31865_cfg_apply(self,MAX31865_CFG_CONVMODE_MASK,state);
	}
}

void MAX31865_cfg_rtd_type(MAX31865_t *self, uint8_t state){

	uint8_t current_cached_config = self->cached_config_register;

	uint8_t current_rtd = current_cached_config & MAX31865_CFG_WIRE_MASK;

	if (current_rtd == state) {
		printf("Already set");
	}
	else{

		MAX31865_cfg_apply(self,MAX31865_CFG_WIRE_MASK,state);
	}
}

void MAX31865_cfg_fault_detection(MAX31865_t *self, uint8_t state){

	uint8_t current_cached_config = self->cached_config_register;

	uint8_t current_fault_md = current_cached_config & MAX31865_CFG_FAULT_DETECT_MASK;

	if (current_fault_md == state) {
		printf("Already set");
	}
	else{

		MAX31865_cfg_apply(self,MAX31865_CFG_FAULT_DETECT_MASK,state);
	}
}

void MAX31865_cfg_filter(MAX31865_t *self, uint8_t state){

	uint8_t current_cached_config = self->cached_config_register;

	uint8_t current_filter = current_cached_config & MAX31865_CFG_FILTER_MASK;

	if (current_filter == state) {
		printf("Already set");
	}
	else{

		MAX31865_cfg_apply(self,MAX31865_CFG_FILTER_MASK,state);
	}

}


void MAX31865_start_one_shot(MAX31865_t *self){

	MAX31865_cfg_apply(self,MAX31865_CFG_VBIAS_MASK,MAX31865_CFG_VBIAS_ON);

	osDelay(5);

	MAX31865_cfg_apply(self,MAX31865_CFG_CONVMODE_MASK,MAX31865_CFG_CONVMODE_N_OFF);

	MAX31865_cfg_apply(self,MAX31865_CFG_ONESHOT_MASK,MAX31865_CFG_ONESHOT_ENABLE);

}


void MAX31865_config_hft(MAX31865_t *self){

}

void MAX31865_config_lft(MAX31865_t *self){

}


void MAX31865_clear_fault(MAX31865_t *self){

	HAL_GPIO_WritePin(self->cs_gpio_port, self->cs_pin, GPIO_PIN_SET);

	uint8_t current_config = (uint8_t)MAX31865_read_register(self, CONFIG_REG, 1);

	uint8_t config_with_clear_strobe = current_config | MAX31865_CFG_FAULT_CLEAR_YES;
	MAX31865_write_register(self, CONFIG_REG, &config_with_clear_strobe);

	uint8_t config_without_clear_strobe = current_config & (~MAX31865_CFG_FAULT_CLEAR_MASK);
	MAX31865_write_register(self, CONFIG_REG, &config_without_clear_strobe);

}

HAL_StatusTypeDef MAX31865_init(MAX31865_t *self, GPIO_TypeDef *cs_gpio_port,uint16_t cs_pin,SPI_HandleTypeDef *spi){

    if (self == NULL || cs_gpio_port == NULL || spi == NULL) {
        printf("MAX31865: Init failed - Invalid NULL pointers.\n");
        return HAL_ERROR; // Return an error if essential pointers are NULL
    }

	memset(self,0,sizeof(*self));

	self->cs_gpio_port = cs_gpio_port ;
	self->cs_pin = cs_pin ;
	self->spi = spi ;

	uint8_t default_config = 	(MAX31865_CFG_VBIAS_ON) |
								(MAX31865_CFG_CONVMODE_AUTO) |
								(MAX31865_CFG_ONESHOT_OFF) |
								(MAX31865_CFG_WIRE_3 ) |
								(MAX31865_CFG_FAULT_RUN_AUTO) |
								(MAX31865_CFG_FILTER_50HZ);


	HAL_GPIO_WritePin(self->cs_gpio_port, self->cs_pin, GPIO_PIN_SET);


	HAL_StatusTypeDef write_status = MAX31865_write_register(self, CONFIG_REG, &default_config);

    if (write_status != HAL_OK) {
        printf("MAX31865: Init failed to write config register.\n");
        return write_status; // Return the specific error from write_register
    }

    printf("MAX31865: Initialized successfully.\n");
    return HAL_OK; // Indicate overall success

}


float MAX31865_read_temperature(MAX31865_t *self){

	MAX31865_clear_fault(self);

	uint16_t adc_rtd_data = 0 ;
	float r_ref = 428.0f ;
	float r_t = 100.0f;
	float rtd = 0 ;
	const float a = 0.00390830f;
	const float b = -0.0000005775f;

	adc_rtd_data = MAX31865_read_register(self, RTD_REG, 2) >> 1;

	rtd = (adc_rtd_data * r_ref) / (1<<15);

	float temperature = (-a + (sqrt((a*a)-(4*b)*(1-(rtd / r_t))))) / (2*b);

	return temperature;

}

void MAX31865_read_fault(MAX31865_t *self){

}

