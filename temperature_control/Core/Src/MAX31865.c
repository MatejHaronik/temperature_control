/*
 * MAX31865.c
 *
 *  Created on: May 2, 2025
 *      Author: mtjha
 */
#include "MAX31865.h"
#include <math.h>
/*MAX31865 Registers*/

#define CONFIG_REG			0x00
#define RTD_MSB_REG			0x01
#define RTD_LSB_REG			0x02
#define HFT_MSB_REG			0x03
#define HFT_LSB_REG			0x04
#define LFT_MSB_REG			0x05
#define LFT_LSB_REG			0x06
#define FAULT_REG			0x07

/*MAX31865 CONFIG_REG bits*/

#define VBIAS_BIT_D7			0x80
#define CONV_MODE_BIT_D6		0x40
#define ONESHOT_BIT_D5			0x20
#define	WIRE_BIT_D4				0x10
#define FAULT_CONTROL_D3		0x08
#define FAULT_CONTROL_D2		0x04
#define CLEAR_FAULT_D1			0x02
#define FREQUENCY_D0			0x01

#define CLEAR_FAULT				1

/*MAX31865 Registers objects*/

MAX31865_Register_t cfg_reg = {.start_register = 0x00, .connected_registers = 1};
MAX31865_Register_t rtd_reg = {.start_register = 0x01, .connected_registers = 2};
MAX31865_Register_t hft_reg = {.start_register = 0x03, .connected_registers = 2};
MAX31865_Register_t lft_reg = {.start_register = 0x04, .connected_registers = 2};
MAX31865_Register_t flt_reg = {.start_register = 0x07, .connected_registers = 1};



uint16_t Read_register(MAX31865_t *max31865, MAX31865_Register_t *reg){

	uint8_t recieved_data_buffer[2] = {0};
	uint8_t tmp = 0xFF;
	uint8_t register_adress = reg->start_register;
	register_adress &= 0x7F;

	HAL_GPIO_WritePin(max31865->cs_gpio_port, max31865->cs_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(max31865->spi, &register_adress, 1, 100);

	for(int i = 0; i < reg->connected_registers; i++){
		HAL_SPI_TransmitReceive(max31865->spi, &tmp, &recieved_data_buffer[i], 1, 100);
	}

	HAL_GPIO_WritePin(max31865->cs_gpio_port, max31865->cs_pin, GPIO_PIN_SET);

	return ((uint16_t)recieved_data_buffer[0] << 8) | recieved_data_buffer[1];

}

void Write_register(MAX31865_t *max31865, MAX31865_Register_t *reg, uint8_t *data){

	uint8_t register_adress = reg->start_register;


	HAL_GPIO_WritePin(max31865->cs_gpio_port, max31865->cs_pin, GPIO_PIN_RESET);
	register_adress |= 0x80;
	HAL_SPI_Transmit(max31865->spi, &register_adress, 1, 100);
	HAL_SPI_Transmit(max31865->spi, data, 1, 100);
	HAL_GPIO_WritePin(max31865->cs_gpio_port, max31865->cs_pin, GPIO_PIN_SET);


}


void Init_MAX31865(MAX31865_t *max31865){

	HAL_GPIO_WritePin(max31865->cs_gpio_port, max31865->cs_pin, GPIO_PIN_SET);
	uint16_t reg = 0;
	HAL_Delay(1000);
	uint8_t config_byte = 	(VBIAS_ON << 7) |				/*D7 - MSB*/
            				(CONVERSION_MODE << 6) |		/*D6*/
							(ONE_SHOT << 5) |				/*D5*/
							(THREE_WIRE << 4) |				/*D4*/
							(FAULT_DETECT_1 << 3) |			/*D3*/
							(FAULT_DETECT_0 << 2) |			/*D2*/
							(FAULT_CLEAR << 1) |			/*D1*/
							(FILTER_TYPE);

	Write_register(max31865,&cfg_reg,&config_byte);
	HAL_Delay(200);
	reg = Read_register(max31865,&cfg_reg);

}

void Clear_fault(MAX31865_t *max31865){

	HAL_GPIO_WritePin(max31865->cs_gpio_port, max31865->cs_pin, GPIO_PIN_SET);
	uint16_t reg = 0;
	HAL_Delay(1000);
	uint8_t config_byte = 	(VBIAS_ON << 7) |				/*D7 - MSB*/
            				(CONVERSION_MODE << 6) |		/*D6*/
							(ONE_SHOT << 5) |				/*D5*/
							(THREE_WIRE << 4) |				/*D4*/
							(FAULT_DETECT_1 << 3) |			/*D3*/
							(FAULT_DETECT_0 << 2) |			/*D2*/
							(CLEAR_FAULT << 1) |			/*D1*/
							(FILTER_TYPE);

	Write_register(max31865,&cfg_reg,&config_byte);
	HAL_Delay(200);
	reg = Read_register(max31865,&cfg_reg);

}

float Read_temperature(MAX31865_t *max31865){
/**
 * call read register and store data to value

 *
**/


	Clear_fault(max31865);
	HAL_Delay(200);

	uint16_t raw_rtd_data = 0 ;

	const float R0 = 100.0f; // PT100
	const float R_REF = 109.8f;
    const float a = 3.9083e-3f;
    const float b = -5.775e-7f;

	raw_rtd_data = Read_register(max31865, &rtd_reg) >> 1;

	float resistance_rtd = ((float)raw_rtd_data / 32768.0f) * R_REF;

	float discriminant = (a * a) - (4 * b * (1 - (resistance_rtd / R0)));


    if (discriminant >= 0.0f) {
        float temperature = (-a + sqrt(discriminant)) / (2 * b);
        return temperature;
    } else {
        // Error: resistance not valid for temperature range
        return -999.0f;
    }

}

