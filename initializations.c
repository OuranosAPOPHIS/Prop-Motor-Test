/*
 * initializations.c
 *
 *  Created on: Jan 20, 2017
 *      Author: Brandon Klefman
 *
 *      Purpose: Function definitions for all initializations of all peripherals.
 *      This will aid in simplicity for the main function.
 */

//*****************************************************************************
//
// Includes
//
//*****************************************************************************
#include "APOPHIS_pin_map.h"
#include "initializations.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

#define APOPHIS false
#define CLOCK_PIOSC 16000000

//*****************************************************************************
//
// External Function Declarations for IntRegister() functions.
//
//*****************************************************************************
extern void SysTickIntHandler(void);
extern void ConsoleIntHandler(void);

/*
 * LED Initialization function.
 */
//*****************************************************************************
//
// This function will initialize the 4 User LED pins. They are pins PN0, PN1
// PF0, and PF4.
//
//*****************************************************************************
void InitLED(uint32_t SysClockSpeed) {
	//
	// Initialize the GPIO port for the LEDs.
	SysCtlPeripheralEnable(LED_GPIO_PERIPH1);
	SysCtlPeripheralEnable(LED_GPIO_PERIPH2);

	//
	// Configure the pins as output pins.
	GPIOPinTypeGPIOOutput(LED_PORT1, LED1_PIN | LED2_PIN);
	GPIOPinTypeGPIOOutput(LED_PORT2, LED3_PIN | LED4_PIN);

	//
	// Initialize a 1 second SysTick for blinking the LED pin 4 to indicate
	// program running.
	SysTickPeriodSet(SysClockSpeed);

	//
	// Register the interrupt handler for blinking the LED and enable it.
	SysTickIntRegister(SysTickIntHandler);
	SysTickIntEnable();
}

/*
 * UART Initialization Functions
 */
//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the program is running.
//
//*****************************************************************************
void InitConsole(void) {
	//
	// Enable GPIO port A which is used for UART0 pins.
	SysCtlPeripheralEnable(CONSOLE_GPIO_PERIPH);

	//
	// Configure the pin muxing for UART0 functions on port A0 and A1.
	GPIOPinConfigure(CONSOLE_CONFIG_PINRX);
	GPIOPinConfigure(CONSOLE_CONFIG_PINTX);

	//
	// Enable UART0 so that we can configure the clock.
	SysCtlPeripheralEnable(CONSOLE_PERIPH);

	//
	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(CONSOLE_UART, UART_CLOCK_PIOSC);

	//
	// Select the alternate (UART) function for these pins.
	GPIOPinTypeUART(CONSOLE_PORT, CONSOLE_PINRX | CONSOLE_PINTX);

	//
	// Initialize the UART for console I/O.
	UARTStdioConfig(0, 115200, CLOCK_PIOSC);

	//
	// Enable the UART interrupt.
	IntEnable(CONSOLE_INT);
	UARTIntEnable(CONSOLE_UART, UART_INT_RX | UART_INT_RT);
	UARTIntRegister(CONSOLE_UART, ConsoleIntHandler);
}

/*
 * Initialization for the PWM module 0 for the
 * air motors. PWM M0 pins 0-3 wll be used.
 */
void InitAirMtrs(uint32_t sysClockSpeed, uint32_t zeroThrottle) {
	uint32_t speed;

	UARTprintf("Initializing air motors...\n\r");

	SysCtlPeripheralDisable(GPIO_PORTK_BASE);
	SysCtlPeripheralReset(GPIO_PORTK_BASE);
	SysCtlPeripheralEnable(GPIO_PORTK_BASE);

	//
	// Wait for the Peripheral to be ready for programming
	while (!SysCtlPeripheralReady(GPIO_PORTK_BASE))
		;

	//
	// Turn on the peripherals for the PWM.
	SysCtlPeripheralEnable(PWM_PERIPHERAL);
	SysCtlPeripheralEnable(PWM_GPIO_PERIPH);

	//
	// Configure the GPIO pins.
	GPIOPinConfigure(GPIO_PF1_M0PWM1);
	GPIOPinConfigure(GPIO_PF2_M0PWM2);
	GPIOPinConfigure(GPIO_PF3_M0PWM3);
	GPIOPinConfigure(GPIO_PG0_M0PWM4);
	GPIOPinTypePWM(PWM_GPIO_PORT1, PWM_MTR_1 | PWM_MTR_2 | PWM_MTR_3);
	GPIOPinTypePWM(PWM_GPIO_PORT2, PWM_MTR_4);

#if !APOPHIS
	//
	// Set up 2 extra motors.
	SysCtlPeripheralEnable(GPIO_PORTK_BASE);

	GPIOPinConfigure(GPIO_PG1_M0PWM5);
	GPIOPinConfigure(GPIO_PK4_M0PWM6);

	GPIOPinTypePWM(GPIO_PORTG_BASE, PWM_MTR_5);
	GPIOPinTypePWM(GPIO_PORTK_BASE, PWM_MTR_6);
#endif

	PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_64);

	//
	// Frequency of PWM.
	speed = (sysClockSpeed / 64 / PWM_FREQUENCY);

	//
	// Configure the PWM.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, speed);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, speed);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, speed);

	uint32_t stuff = PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0);

	UARTprintf("PWM generator period: %d\r\n", stuff);

#if !APOPHIS
	//
	// Set up the generator for the 6th motor.
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, speed);
#endif

	//
	// Initialize pulse to 5%
	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_1, zeroThrottle);
	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_2, zeroThrottle);
	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_3, zeroThrottle);
	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_4, zeroThrottle);

#if !APOPHIS
	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_5, zeroThrottle);
	PWMPulseWidthSet(PWM0_BASE, MOTOR_OUT_6, zeroThrottle);

	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT |
	PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);

	UARTprintf("Motors initialized for test rig.\r\nDone!\n\r");
#else

	//
	// Set the output PWM modules.
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_3_BIT |
			PWM_OUT_4_BIT, true);

	UARTprintf("Motors initialized for APOPHIS.\r\nDone!\n\r");
#endif
}

