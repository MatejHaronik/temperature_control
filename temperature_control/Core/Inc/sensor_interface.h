/*
 * sensor_interface.h
 *
 *  Created on: Jun 17, 2025
 *      Author: mtjha
 */

#ifndef INC_SENSOR_INTERFACE_H_
#define INC_SENSOR_INTERFACE_H_

typedef enum {

    SENSOR_STATUS_OK = 0,
    SENSOR_STATUS_ERROR,             // Generic error
    SENSOR_STATUS_NOT_INITIALIZED,   // Sensor not initialized or failed initialization
    SENSOR_STATUS_READ_FAILED,       // Failed to read data (e.g., communication error, invalid data)
    SENSOR_STATUS_INVALID_PARAM,     // Invalid parameter passed to a function
    SENSOR_STATUS_DEINIT_FAILED,     // Deinitialization failed
    SENSOR_STATUS_MANAGER_FULL,      // For manager-level errors: no more room to register sensors
    SENSOR_STATUS_NOT_FOUND,

}SensorStatus_t;

typedef struct {


}SensorInterface_t;

#endif /* INC_SENSOR_INTERFACE_H_ */
