/*
 * parser.h
 *
 *  Created on: Nov 10, 2015
 *      Author: Mallesh
 */

#ifndef PARSER_H_
#define PARSER_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"

	char* str_ncpy(char* ,char* ,uint32_t );
    uint32_t str_len(char*);

    void transfer(char*, uint32_t);
	void dec_ascii(uint32_t,char*);
	uint32_t ascii_dec(char* ,uint32_t*);
	uint32_t ascii_hex_dec(char*,uint32_t*);
	uint32_t int_hex_ascii(char* , uint32_t);

#endif /* PARSER_H_ */
