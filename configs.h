/*
 * configs.h
 *
 *  Created on: 02-Mar-2016
 *      Author: Mallesh
 */

#ifndef CONFIGS_H_
#define CONFIGS_H_
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"

void InitI2C(uint32_t , bool);
void InitConsole();
void InitConsole1();
void InitConsole3();


#endif /* CONFIGS_H_ */
