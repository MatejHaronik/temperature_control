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

#define VBIAS_ON							1	/*1 - ON ; 2 - OFF */
#define CONVERSION_MODE						1	/*1 - AUTO; 0 - Normally OFF*/
#define ONE_SHOT							0	/*1 - 1-shot*/
#define THREE_WIRE							1	/*1- 3-wire RTD ; 0 - 2 or 4 wire RTD*/
#define FAULT_DETECT_AUTO					0	/*Combination of bits 0 1 to set this auto*/
#define FAULT_CLEAR							0	/*Write a 1 to this bit while writing 0 to bits D5, D3, and D2 to return all fault status bits (D[7:2])*/
#define FILTER_TYPE							0	/*1-50Hz; 0-60Hz*/

#if FAULT_DETECT_AUTO
	#define	FAULT_DETECT_1		0
	#define	FAULT_DETECT_0		1
#else
	#define	FAULT_DETECT_1		0
	#define	FAULT_DETECT_0		0
#endif



typedef struct {
    uint8_t start_register;
    uint8_t connected_registers;
} MAX31865_Register_t;


typedef struct{

GPIO_TypeDef		*cs_gpio_port;
uint16_t			cs_pin;
SPI_HandleTypeDef	*spi;

}MAX31865_t;

void Init_MAX31865(MAX31865_t *max31865);
float Read_temperature(MAX31865_t *max31865);


#endif /* INC_MAX31865_H_ */
