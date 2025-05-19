/*
 * slave_modbus_rtu.c
 *
 *  Created on: May 11, 2025
 *      Author: mtjha
 */
#include "modbus_rtu.h"
#include <stdbool.h>



uint8_t supported_function_codes[NUM_SUPPORTED_FUNCTIONS] =
{READ_COILS, READ_DISCRETE_INPUT, READ_HOLDING_REGISTERS, READ_INPUT_REGISTERS,WRITE_SINGLE_COIL,WRITE_SINGLE_REGISTER,READ_EXCEPCION_STATUS,DIAGNOSTICS,0x09,
 0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,
 0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,
 0x30
};

uint16_t holding_register_map[NUM_HOLDING_REGISTERS];

extern ring_buffer rx_buffer;
extern ring_buffer tx_buffer;


uint16_t Rx_crc16_calculate(/*uint8_t*/ ring_buffer *ring_buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    uint8_t *buffer = ring_buffer->buffer;
    uint8_t buffer_pos = ring_buffer->head;

    buffer_pos = buffer_pos - (length + 2);


    for (int pos = 0; pos < length; pos++) {
    	uint8_t actual_pos = (buffer_pos + pos) % UART_BUFFER_SIZE;
        crc ^= (uint16_t)buffer[actual_pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else
                crc >>= 1;
        }
    }
    return crc;
}

uint16_t Tx_crc16_calculate(uint8_t *message, uint16_t length) {

	uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < length - 2; pos++) {
        crc ^= (uint16_t)message[pos];
        for (int i = 8; i != 0 ; i--) {
            if ((crc & 0x0001) != 0){
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;

        }
    }
    return crc;
}

uint8_t Function_code_check(uint8_t function_code) {

	uint8_t status = 0;

	for (int i = 0; i < NUM_SUPPORTED_FUNCTIONS; i++) {

		if(function_code == supported_function_codes[i]){
			status = 1;
			break;
		}
		else
			status = 0 ;
	}

	return status;
}

void Move_from_ring_buffer_to_modbus_handler(ring_buffer *ring_buffer, uint16_t length, uint8_t *buffer){

	for (int i = 0; i < length; i++) {

		buffer[i] = (ring_buffer->buffer[(ring_buffer->head) - length + i] );
		ring_buffer->buffer[(ring_buffer->head) - length + i] = 0 ;
	}

	ring_buffer->head -= length ;
}

void Read_holding_register_handle(uint8_t *modbus_message){

	uint16_t start_adress = modbus_message [2] << 8 | modbus_message [3];
	uint16_t no_of_registers = modbus_message [4] << 8 | modbus_message [5];
	uint8_t message_hi_bit = 0;
	uint8_t message_low_bit = 0;
	uint8_t message_length = (no_of_registers * 2) + 5;
	uint8_t message_response[message_length];
	uint8_t message_index = 3;	/*starting adding values from third bite of response array*/
	uint16_t crc = 0;

	message_response[0] = modbus_message[0];

	message_response[1] = modbus_message[1];

	message_response[2] = no_of_registers * 2;

	for (int i = 0; i < no_of_registers; i++) {

		message_hi_bit = (holding_register_map[start_adress + i] >> 8) & 0xFF;
		message_response[message_index] = message_hi_bit;
		message_index++;
		message_low_bit = holding_register_map[start_adress + i] & 0xFF;
		message_response[message_index] = message_low_bit;
		message_index++;
	}

	crc = Tx_crc16_calculate(message_response,message_length);
	message_response[message_length - 2] = (crc >> 8) & 0xFF;
	message_response[message_length -1 ] = crc & 0xFF;

	for (int i = 0; i < message_length; i++) {
		Uart_write(message_response[i]);
	}



}

void Process_message_request(uint8_t *modbus_message){

	uint8_t function_code = modbus_message[1];

	switch (function_code){

	case READ_HOLDING_REGISTERS:

		Read_holding_register_handle(modbus_message);

		break;
	}




}

void Modbus_recieve_message_handle(ring_buffer *ring_buffer, uint16_t length) {

	uint8_t modbus_message[length] ;

    if (length < 5) {
        printf("Frame too short.\n");

        return;
    }

    uint16_t crc_calc = Rx_crc16_calculate(ring_buffer, length - 2);
    uint16_t crc_recv = (ring_buffer->buffer[(ring_buffer->head) -1 ])  << 8 | (ring_buffer->buffer[(ring_buffer->head) -2 ]);


    if (crc_calc != crc_recv) {
        printf("CRC mismatch! Calculated: 0x%04X, Received: 0x%04X\n", crc_calc, crc_recv);

        return;
    }

    Move_from_ring_buffer_to_modbus_handler(ring_buffer, length, modbus_message);

    printf("Slave ID: %d\n", modbus_message[0]);
    printf("Function: 0x%02X\n", modbus_message[1]);

    if (modbus_message[0] != SLAVE_ID ) {

        printf("Message for diferent slave.\n");
        return;
    }

    if (!Function_code_check(modbus_message[1])){
        printf("Unsupported function code.\n");
        return;
    }

    Process_message_request(modbus_message);

}

void Modbus_transmit_message_handle(ring_buffer *ring_buffer, uint16_t length){


}
