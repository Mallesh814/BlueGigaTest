
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
#include "cmd_def.h"
#include "uart.h"

uint8_t uart_char = 0, call_parser = 0, tick = 0;
uint8_t uart_char3 = 0, ble_data = 0, console_event = 0;
void Timer0AIntHandler(void);

#define MAX_DEVICES 64

int found_devices_count = 0;
bd_addr found_devices[MAX_DEVICES];
bd_addr connect_addr;

enum actions {
    action_none,
    action_scan,
    action_connect,
    action_info,
};
enum actions action = action_none;

typedef enum {
    state_disconnected,
    state_connecting,
    state_connected,
    state_finding_services,
    state_finding_attributes,
    state_listening_measurements,
    state_finish,
    state_last
} states;
states state = state_disconnected;

const char *state_names[state_last] = {
    "disconnected",
    "connecting",
    "connected",
    "finding_services",
    "finding_attributes",
    "listening_measurements",
    "finish"
};

#define FIRST_HANDLE 0x0001
#define LAST_HANDLE  0xffff

#define THERMOMETER_SERVICE_UUID            0x1809
#define THERMOMETER_MEASUREMENT_UUID        0x2a1c
#define THERMOMETER_MEASUREMENT_CONFIG_UUID 0x2902

uint8 primary_service_uuid[] = {0x00, 0x28};

uint16 thermometer_handle_start = 0,
       thermometer_handle_end = 0,
       thermometer_handle_measurement = 0,
       thermometer_handle_configuration = 0;


void change_state(states new_state)
{
#ifdef DEBUG
    transfer("DEBUG: State changed: ", UART0_BASE);
    transfer(state_names[state], UART0_BASE);
    transfer("--> ", UART0_BASE);
    transfer(state_names[new_state], UART0_BASE);
    transfer("\n\r", UART0_BASE);
#endif
    state = new_state;
}

int cmp_bdaddr(bd_addr first, bd_addr second)
{
    int i;
    for (i = 0; i < sizeof(bd_addr); i++) {
        if (first.addr[i] != second.addr[i]) return 1;
    }
    return 0;
}

void print_bdaddr(bd_addr bdaddr)
{
	uint32_t i = 0;
	char num[3]="\0";

	for(i = 0; i < 6; i++){
		int_hex_ascii(num, bdaddr.addr[5 - i]);
	    transfer(num, UART0_BASE);
	    transfer(":", UART0_BASE);
	}
    transfer("\n\r", UART0_BASE);

}

void enable_indications(uint8 connection_handle, uint16 client_configuration_handle)
{
    uint8 configuration[] = {0x02, 0x00}; // enable indications
    ble_cmd_attclient_attribute_write(connection_handle, thermometer_handle_configuration, 2, &configuration);
}


void ble_rsp_system_get_info(const struct ble_msg_system_get_info_rsp_t *msg)
{
	char num[10]="\0";
	transfer("Build: ", UART0_BASE);

    dec_ascii(msg->build , num);
    transfer(num, UART0_BASE);
    transfer("\n\r", UART0_BASE);

    transfer(", protocol_version: ", UART0_BASE);
	dec_ascii(msg->protocol_version , num);
    transfer(num, UART0_BASE);
    transfer("\n\r", UART0_BASE);

    transfer(", hardware: ", UART0_BASE);


    switch (msg->hw) {
    case 0x01: transfer("BLE112", UART0_BASE); break;
    case 0x02: transfer("BLED112", UART0_BASE); break;
    default: transfer("Unknown", UART0_BASE);
    }
    transfer("\n", UART0_BASE);

    if (action == action_info) change_state(state_finish);
}

void ble_evt_gap_scan_response(const struct ble_msg_gap_scan_response_evt_t *msg)
{
    if (found_devices_count >= MAX_DEVICES) change_state(state_finish);

    int i;
	char num[10]="\0";
	char payload[64];
    char *name = &payload[0];

    // Check if this device already found
    for (i = 0; i < found_devices_count; i++) {
        if (!cmp_bdaddr(msg->sender, found_devices[i])) return;
    }
    found_devices_count++;
    mem_cpy(found_devices[i].addr, msg->sender.addr, sizeof(bd_addr));

    // Parse data
    for (i = 0; i < msg->data.len; ) {
        int8 len = msg->data.data[i++];
        if (!len) continue;
        if (i + len > msg->data.len) break; // not enough data
        uint8 type = msg->data.data[i++];
        switch (type) {
        case 0x09:
//            name = malloc(len);
            mem_cpy(name, msg->data.data + i, len - 1);
            name[len - 1] = '\0';
        }

        i += len - 1;
    }

    print_bdaddr(msg->sender);
    transfer(" RSSI:", UART0_BASE);

	dec_ascii(msg->rssi , num);
    transfer(num, UART0_BASE);
    transfer("\n\r", UART0_BASE);



    transfer(" Name:", UART0_BASE);
    if (name) transfer(name, UART0_BASE);
    else transfer("Unknown", UART0_BASE);
    transfer("\n\r", UART0_BASE);

//    free(name);
}

void ble_evt_connection_status(const struct ble_msg_connection_status_evt_t *msg)
{
    // New connection
    if (msg->flags & connection_connected) {
        change_state(state_connected);
        transfer("Connected\n", UART0_BASE);

        // Handle for Temperature Measurement configuration already known
        if (thermometer_handle_configuration) {
            change_state(state_listening_measurements);
            enable_indications(msg->connection, thermometer_handle_configuration);
        }
        // Find primary services
        else {
            change_state(state_finding_services);
            ble_cmd_attclient_read_by_group_type(msg->connection, FIRST_HANDLE, LAST_HANDLE, 2, primary_service_uuid);
        }
    }
}

void ble_evt_attclient_group_found(const struct ble_msg_attclient_group_found_evt_t *msg)
{
    if (msg->uuid.len == 0) return;
    uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

    // First thermometer service found
    if (state == state_finding_services && uuid == THERMOMETER_SERVICE_UUID && thermometer_handle_start == 0) {
        thermometer_handle_start = msg->start;
        thermometer_handle_end = msg->end;
    }
}

void ble_evt_attclient_procedure_completed(const struct ble_msg_attclient_procedure_completed_evt_t *msg)
{
    if (state == state_finding_services) {
        // Thermometer service not found
        if (thermometer_handle_start == 0) {
            transfer("No Health Thermometer service found\n", UART0_BASE);
            change_state(state_finish);
        }
        // Find thermometer service attributes
        else {
            change_state(state_finding_attributes);
            ble_cmd_attclient_find_information(msg->connection, thermometer_handle_start, thermometer_handle_end);
        }
    }
    else if (state == state_finding_attributes) {
        // Client characteristic configuration not found
        if (thermometer_handle_configuration == 0) {
            transfer("No Client Characteristic Configuration found for Health Thermometer service\n", UART0_BASE);
            change_state(state_finish);
        }
        // Enable temperature notifications
        else {
            change_state(state_listening_measurements);
            enable_indications(msg->connection, thermometer_handle_configuration);
        }
    }
}

void ble_evt_attclient_find_information_found(const struct ble_msg_attclient_find_information_found_evt_t *msg)
{
    if (msg->uuid.len == 2) {
        uint16 uuid = (msg->uuid.data[1] << 8) | msg->uuid.data[0];

        if (uuid == THERMOMETER_MEASUREMENT_UUID) {
            thermometer_handle_measurement = msg->chrhandle;
        }
        else if (uuid == THERMOMETER_MEASUREMENT_CONFIG_UUID) {
            thermometer_handle_configuration = msg->chrhandle;
        }
    }
}

#define THERMOMETER_FLAGS_FAHRENHEIT 0x1
void ble_evt_attclient_attribute_value(const struct ble_msg_attclient_attribute_value_evt_t *msg)
{
    char num[10]="\0";
	if (msg->value.len < 5) {
        transfer("Not enough fields in Temperature Measurement value", UART0_BASE);
        change_state(state_finish);
    }

    uint8 flags = msg->value.data[0];
    int8 exponent = msg->value.data[4];
    int mantissa = (msg->value.data[3] << 16) | (msg->value.data[2] << 8) | msg->value.data[1];

    if (exponent >= 0)
        exponent = 0;
    else
        exponent = (exponent < 0) ? (exponent * -1):(exponent) ;

    transfer("Temperature: \n\r", UART0_BASE);
    transfer("Mantissa: \n\r", UART0_BASE);
	dec_ascii(mantissa , num);
    transfer(num, UART0_BASE);
    transfer("\n\r", UART0_BASE);
    transfer("Exponent: \n\r", UART0_BASE);
	dec_ascii(exponent , num);
    transfer(num, UART0_BASE);
    transfer("\n\r", UART0_BASE);

    if (flags & THERMOMETER_FLAGS_FAHRENHEIT)
        transfer("F", UART0_BASE);
    else
        transfer("C", UART0_BASE);
    transfer("\n\r", UART0_BASE);
}

void ble_evt_connection_disconnected(const struct ble_msg_connection_disconnected_evt_t *msg)
{
    change_state(state_disconnected);
    transfer("Connection terminated, trying to reconnect\n\r", UART0_BASE);
    change_state(state_connecting);
    ble_cmd_gap_connect_direct(&connect_addr, gap_address_type_public, 40, 60, 100,0);
}

void output(uint8 len1, uint8* data1, uint16 len2, uint8* data2)
{
	uint8_t dat;
	dat = len1+len2;
    if (uart_tx(1, &dat) || uart_tx(len1, data1) || uart_tx(len2, data2)) {
        transfer("ERROR: Writing to serial port failed\n", UART0_BASE);
    }
}


int main(void)
{

	char tx_str[] = "0123456789ABCDEF";

	extern uint32_t serial_handle;


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

    bglib_output = output;
    serial_handle = UART3_BASE;

    //
    // Display the example setup on the console.
    //
    transfer("Initialization Done\n\r", UART0_BASE);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // The Timer0 peripheral must be enabled for use.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //
    // Set the Timer0B load value to 0.625ms.
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 1);

    //TimerControlWaitOnTrigger(TIMER0_BASE, TIMER_B, true);
    //
    // Configure the Timer0B interrupt for timer timeout.
    //
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    //
    // Enable the Timer0B interrupt on the processor (NVIC).
    //
    IntEnable(INT_TIMER0A);

    //
    // Enable Timer0B.
    //
    TimerEnable(TIMER0_BASE, TIMER_A );

    transfer("Timer Started\n\r", UART0_BASE);

//    ble_cmd_system_reset(0);
	char num[3]="\0";

//    transfer(tx_str, UART3_BASE);

    while (1){

    	if(call_parser){
    		call_parser = 0;
    		ble_cmd_system_address_get();
    	    transfer("S D \n\r", UART0_BASE);
    	}

    	if(console_event){
    		console_event = 0;
    		switch(uart_char){
    		case 'r':	ble_cmd_system_reset(0);
    					break;
    		case 'm':	ble_cmd_system_address_get();
    					break;
    		case 'g':	ble_cmd_gap_set_mode(2,2);
    					break;
/*    		case 'r':	ble_cmd_system_reset(0);
    					break;
*/    		default :	ble_cmd_system_hello();
    					break;


    		}
    	}
    	if(ble_data){
    		ble_data = 0;
    	    transfer("S\n\r", UART0_BASE);
    	    while(UARTCharsAvail(UART3_BASE)){
    	    	uart_char3 = UARTCharGet(UART3_BASE);
    	    	int_hex_ascii(num, uart_char3);
				transfer(num, UART0_BASE);
				transfer(":", UART0_BASE);
    	    }
	        transfer("\n\r", UART0_BASE);

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
			console_event = 1;
		}
}

void isr_uart3()
{
	uint32_t u3status;
	u3status = UARTIntStatus(UART3_BASE,true);
	UARTIntClear(UART3_BASE,u3status);
	if(UARTCharsAvail(UART3_BASE))
		{
			ble_data = 1;
			GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1, uart_char3);	// Toggle LED0 everytime a key is pressed
		}
}

//*****************************************************************************
//
// The interrupt handler for the Timer0B interrupt.
//
//*****************************************************************************
void
Timer0AIntHandler(void)
{
    //
    // Clear the timer interrupt flag.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//    call_parser=1;
}

