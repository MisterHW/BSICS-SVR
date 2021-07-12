/**
  ******************************************************************************
  * File Name          : devices.h
  * Description        : BSICS Server peripheral devices wrapper.
  *
  ******************************************************************************
  *
  *************************************************************************

  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef BSICS_SVR_DEVICES_H
#define BSICS_SVR_DEVICES_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Global Variables ----------------------------------------------------------*/

/* Class Declarations  -------------------------------------------------------*/
#ifdef __cplusplus
class TPeripheralDeviceGroup {
    I2C_HandleTypeDef hI2C;

public:
    bool init(I2C_HandleTypeDef _hI2C);
};

typedef TPeripheralDeviceGroup* hPeripheralDeviceGroup;
#endif

/* Functions -----------------------------------------------------------------*/

bool Devices_Init(void);

/* ---------------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif /* BSICS_SVR_DEVICES_H */

/**** END OF FILE ****/
