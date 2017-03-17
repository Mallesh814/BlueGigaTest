/*
 * configs.c
 *
 *  Created on: 02-Mar-2016
 *      Author: Mallesh
 */
#include "configs.h"

extern void isr_uart();// rst_handler contains the code to run on reset.
extern void isr_uart1();// rst_handler contains the code to run on reset.
extern void isr_uart3();// rst_handler contains the code to run on reset.


void InitI2C(uint32_t BASE, bool mode)
{
	uint32_t SYSCTL_PERIPH_I2C, SYSCTL_PERIPH_GPIO;
	uint32_t CONF_PIN_SCL, CONF_PIN_SDA;
	uint32_t GPIO_PORT_BASE;
	uint32_t PIN_SCL, PIN_SDA;
	switch(BASE)
	{
		case I2C0_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C0;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOB;
						CONF_PIN_SCL = GPIO_PB2_I2C0SCL;
						CONF_PIN_SDA = GPIO_PB3_I2C0SDA;
						GPIO_PORT_BASE = GPIO_PORTB_BASE;
						PIN_SCL = GPIO_PIN_2;
						PIN_SDA = GPIO_PIN_3;
						break;

		case I2C1_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C1;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOA;
						CONF_PIN_SCL = GPIO_PA6_I2C1SCL;
						CONF_PIN_SDA = GPIO_PA7_I2C1SDA;
						GPIO_PORT_BASE = GPIO_PORTA_BASE;
						PIN_SCL = GPIO_PIN_6;
						PIN_SDA = GPIO_PIN_7;
						break;

		case I2C2_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C2;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOE;
						CONF_PIN_SCL = GPIO_PE4_I2C2SCL;
						CONF_PIN_SDA = GPIO_PE5_I2C2SDA;
						GPIO_PORT_BASE = GPIO_PORTE_BASE;
						PIN_SCL = GPIO_PIN_4;
						PIN_SDA = GPIO_PIN_5;
						break;

		case I2C3_BASE:	SYSCTL_PERIPH_I2C  =  SYSCTL_PERIPH_I2C3;
						SYSCTL_PERIPH_GPIO =  SYSCTL_PERIPH_GPIOD;
						CONF_PIN_SCL = GPIO_PD0_I2C3SCL;
						CONF_PIN_SDA = GPIO_PD1_I2C3SDA;
						GPIO_PORT_BASE = GPIO_PORTD_BASE;
						PIN_SCL = GPIO_PIN_0;
						PIN_SDA = GPIO_PIN_1;
						break;

		default:		while(1);

	}

	// Enable I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C);
	//Wait Until I2C is Ready
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_I2C)));

	//Enable GPIO
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO);
	//Wait Until GPIO is Ready
	while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIO)));


	GPIOPinConfigure(CONF_PIN_SCL);
	GPIOPinConfigure(CONF_PIN_SDA);

	GPIOPinTypeI2C(GPIO_PORT_BASE, PIN_SDA);
	GPIOPinTypeI2CSCL(GPIO_PORT_BASE, PIN_SCL);

	I2CMasterInitExpClk(BASE,SysCtlClockGet(),mode);

}

void InitConsole()
{

	uint32_t status = 0;

	//
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
    // Enable UART0 so that we can configure the clock.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinTypeUART(GPIO_PORTA_BASE,GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTFIFODisable(UART0_BASE);

	UARTFIFOEnable(UART0_BASE);
	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX7_8,UART_FIFO_RX1_8);

	UARTIntRegister(UART0_BASE,isr_uart);
	IntEnable(INT_UART0);

	UARTIntEnable(UART0_BASE,UART_INT_RX | UART_INT_RT);

	status = UARTIntStatus(UART0_BASE,true);
	UARTIntClear(UART0_BASE,status);
}


void InitConsole1()
{

	uint32_t status = 0;

	//
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//
    // Enable UART0 so that we can configure the clock.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinTypeUART(GPIO_PORTB_BASE,GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTFIFODisable(UART1_BASE);

	//	UARTFIFOEnable(UART0_BASE);
	//	UARTFIFOLevelSet(UART0_BASE,UART_FIFO_TX7_8,UART_FIFO_RX1_8);

	UARTIntRegister(UART1_BASE,isr_uart1);
	IntEnable(INT_UART1);

	UARTIntEnable(UART1_BASE,UART_INT_RX);

	status = UARTIntStatus(UART1_BASE,true);
	UARTIntClear(UART1_BASE,status);
}

void InitConsole3()
{

	uint32_t status = 0;

	//
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	//
    // Enable UART0 so that we can configure the clock.
    //
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinConfigure(GPIO_PC7_U3TX);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
	GPIOPinTypeUART(GPIO_PORTC_BASE,GPIO_PIN_6 | GPIO_PIN_7);

    //
    // Initialize the UART for console I/O.
    //
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTFIFODisable(UART3_BASE);

	UARTFIFOEnable(UART3_BASE);
	UARTFIFOLevelSet(UART3_BASE,UART_FIFO_TX7_8,UART_FIFO_RX6_8);

	UARTIntRegister(UART3_BASE,isr_uart3);
	IntEnable(INT_UART3);

	UARTIntEnable(UART3_BASE,UART_INT_RX | UART_INT_RT );

	status = UARTIntStatus(UART3_BASE,true);
	UARTIntClear(UART3_BASE,status);
}


