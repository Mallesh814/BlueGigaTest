
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "driverlib/timer.h"
#include "uartstdio.h"

#include "configs.h"
#include "parser.h"

uint8_t uart_char = 0, call_parser = 0, tick = 0;
uint8_t uart_char3 = 0, call_parser1 = 0;

int main(void)
{

	char tx_str[] = "0123456789ABCDEF";

	uint32_t command;


	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, 0X00);	// Toggle LED0 everytime a key is pressed

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

	//	UART Configuration
    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for Timer operation.
    //
    InitConsole();
    InitConsole3();
    //
    // Display the example setup on the console.
    //
    transfer("Initialization Done\n\r", UART0_BASE);
    transfer(tx_str, UART3_BASE);

    while (1){
    	if(call_parser){
    		call_parser = 0;
    	    transfer(tx_str, UART3_BASE);

    	}

    }


}

void isr_uart()
{
	uint32_t u0status;
	u0status = UARTIntStatus(UART0_BASE,true);
	UARTIntClear(UART0_BASE,u0status);
	if(UARTCharsAvail(UART0_BASE))
		{
			uart_char = UARTCharGet(UART0_BASE);
			call_parser = 1;
		}
}

void isr_uart3()
{
	uint32_t u3status;
	u3status = UARTIntStatus(UART3_BASE,true);
	UARTIntClear(UART3_BASE,u3status);
	if(UARTCharsAvail(UART3_BASE))
		{
			uart_char3 = UARTCharGet(UART3_BASE);
			UARTCharPut(UART0_BASE, uart_char3);
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, uart_char3);	// Toggle LED0 everytime a key is pressed
		}
}

