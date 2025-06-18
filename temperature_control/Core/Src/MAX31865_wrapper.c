
#include "MAX31865_wrapper.h" // Includes sensor_interface.h and MAX31865.h
#include <stdio.h>            // For printf (for debugging, replace in production)
#include <stddef.h>

static MAX31865_t _max31865_driver_instance;
static SensorInterface_t _max31865_generic_interface;

static SensorStatus_t MAX31865_init_wrapper(void *driver_data_ptr) {
    // 1. Cast the generic 'void*' pointer back to the specific MAX31865_t* type.
    MAX31865_t *dev = (MAX31865_t *)driver_data_ptr;

    if (dev == NULL || dev->cs_gpio_port == NULL || dev->spi == NULL) {
        printf("MAX31865 Wrapper: Init failed - Invalid driver data or unpopulated handles.\n");
        return SENSOR_STATUS_INVALID_PARAM;
    }


    HAL_StatusTypeDef hal_status = MAX31865_init(
        dev,                 // 'self' parameter in MAX31865_init
        dev->cs_gpio_port,   // 'cs_gpio_port' parameter in MAX31865_init
        dev->cs_pin,         // 'cs_pin' parameter in MAX31865_init
        dev->spi             // 'spi' parameter in MAX31865_init
    );

    if (hal_status == HAL_OK) {
        printf("MAX31865 Wrapper: Specific init OK.\n");
        return SENSOR_STATUS_OK;
    } else {
        printf("MAX31865 Wrapper: Specific init FAILED (HAL Status: %d).\n", (int)hal_status);
        return SENSOR_STATUS_NOT_INITIALIZED; // Or map to a more specific error
    }

}
