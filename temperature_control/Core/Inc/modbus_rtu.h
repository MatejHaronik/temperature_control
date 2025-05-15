/*
 * slave_modbus_rtu.h
 *
 *  Created on: May 11, 2025
 *      Author: mtjha
 */

#ifndef INC_MODBUS_RTU_H_
#define INC_MODBUS_RTU_H_

#include "main.h"
#include "uart_ring_buffer.h"

#define NUM_SUPPORTED_FUNCTIONS 30

#define NUM_HOLDING_REGISTERS	10

#define MASTER_MODE 		0		/*Library is versatile for both usage. MASTER_MODE 0 should be used for device in slave mode*/

#if MASTER_MODE == 0
	#define SLAVE_ID		0x01
#endif

#define READ_COILS								0x01
#define READ_DISCRETE_INPUT						0x02
#define READ_HOLDING_REGISTERS					0x03
#define READ_INPUT_REGISTERS					0x04
#define WRITE_SINGLE_COIL						0x05
#define WRITE_SINGLE_REGISTER					0x06
#define READ_EXCEPCION_STATUS					0x07
#define DIAGNOSTICS								0x08



void Modbus_recieve_message_handle(ring_buffer *ring_buffer, uint16_t length);





#endif /* INC_MODBUS_RTU_H_ */
