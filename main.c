
#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "inc/hw_memmap.h"
#include "driverlib/timer.h"

#include "configs.h"
#include "parser.h"
#include "ble113.h"



uint8_t uart_char = 0, call_parser = 0, tick = 0;
uint8_t uart_char3 = 0, ble_data = 0, console_event = 0;

void isr_debugConsole();// rst_handler contains the code to run on reset.
void isr_bleConsole();


uint32_t debugConsole;
extern uint32_t bleConsole;


int main(void)
{

	char tx_str[] = "0123456789ABCDEF";
	char num[3]="\0";
	uint32_t status;
    bglib_output = output;

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
    debugConsole = InitConsole(UART0_BASE,115200);
	UARTFIFOEnable(debugConsole);
	UARTFIFOLevelSet(debugConsole,UART_FIFO_TX7_8,UART_FIFO_RX1_8);
	UARTIntRegister(debugConsole,isr_debugConsole);
	UARTIntEnable(debugConsole,UART_INT_RX | UART_INT_RT);
	status = UARTIntStatus(debugConsole,true);
	UARTIntClear(debugConsole,status);

	bleConsole = InitConsole(UART3_BASE,115200);
	UARTFIFOEnable(bleConsole);
	UARTFIFOLevelSet(bleConsole,UART_FIFO_TX7_8,UART_FIFO_RX6_8);
	UARTIntRegister(bleConsole,isr_bleConsole);
	UARTIntEnable(bleConsole,UART_INT_RX | UART_INT_RT );
	status = UARTIntStatus(bleConsole,true);
	UARTIntClear(bleConsole,status);

    //
    // Display the example setup on the console.
    //
    transfer("Initialization Done\n\r", debugConsole);
    TimerConfig(1);
    transfer("Timer Started\n\r", debugConsole);

    while (1){

    	if(call_parser){
    		call_parser = 0;
    		ble_cmd_system_address_get();
    	    transfer("S D \n\r", debugConsole);
    	}

    	if(console_event){
    		console_event = 0;
    		switch(uart_char){
    		case 'r':	ble_cmd_system_reset(0);
    					break;
    		case 'm':	ble_cmd_system_address_get();
    					break;
    		case 'g':	ble_cmd_gap_set_mode(2,2);
    					change_state(state_advertising);
    					break;
/*    		case 'r':	ble_cmd_system_reset(0);
    					break;
*/    		default :	ble_cmd_system_hello();
    					break;


    		}
    	}
    	if(ble_data){
    		ble_data = 0;
    	    transfer("S\n\r", debugConsole);
    	    read_message(1000);
    	    transfer("\n\r", debugConsole);
    	}
    }
}

void isr_debugConsole()
{
	uint32_t u0status;
	u0status = UARTIntStatus(debugConsole,true);
	UARTIntClear(debugConsole,u0status);
	if(UARTCharsAvail(debugConsole))
		{
			uart_char = UARTCharGet(debugConsole);
			console_event = 1;
		}
}

void isr_bleConsole()
{
	uint32_t u3status;
	u3status = UARTIntStatus(bleConsole,true);
	UARTIntClear(bleConsole,u3status);
	if(UARTCharsAvail(bleConsole))
		{
			ble_data = 1;
//			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, uart_char3);	// Toggle LED0 everytime a key is pressed
		}
}
void Timer0AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//    call_parser=1;
}

