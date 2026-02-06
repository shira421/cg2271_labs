/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_3.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/*
 * @brief   Application entry point.
 */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

#define SWITCH_PIN   4  // PTA4

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

/*
 * @brief   Application entry point.
 */
// Configure ADC_SE0 on PTE20 and ADC_SE4a on PTE21

#define ADC_SE0			0
#define ADC_SE0_PIN	20
#define ADC_SE4a			4
#define ADC_SE4_PIN	21

//TODO
void initADC() {
}

int result[2];

void startADC(int channel) {
}

//TODO! This is NOT going to work.
void ADCX_IRQHandler(){
	static int turn=0;

	NVIC_ClearPendingIRQ(ADCX_IRQn);
	if(ADCX->SC1[0] & ADC_SC1_COCO_MASK) {
		result[turn] = ADCX->R[0];
		//PRINTF("Turn = %d, Result = %d\r\n", turn, result[turn]);
		turn = 1 - turn;
		if(turn == 0) {
			startADC(ADC_XXX);
		} else {
			startADC(ADC_YYY);
		}
	}
}

// Configure the MCG Internal Reference Clock
void setMCGIRClk() {
}

void setTPMClock(){
}

void initPWM() {
}

void startPWM() {
}

void stopPWM() {
}

void setPWM(int LED, int percent) {
}

void initButton() {
    //Disable interrupts
    NVIC_DisableIRQ(PORTA_IRQn);

    //Enable clock gating to PORTA
    SIM->SCGC5 |= (SIM_SCGC5_PORTA_MASK);

    //Configure MUX of PTA4
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_MUX(1);

	//Set pullup resistor
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PS_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PS(1);
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_PE_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_PE(1);

	//Set as input
	GPIOA->PDDR &= ~(1 << SWITCH_PIN);

	//Configure the interrupt for falling edge
	PORTA->PCR[SWITCH_PIN] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[SWITCH_PIN] |= PORT_PCR_IRQC(0b1010);

	//Set NVIC priority to 0,
	//highest priority
	NVIC_SetPriority(PORTA_IRQn, 0);


	//Clear pending interrupts and enable interrupts
	NVIC_ClearPendingIRQ(PORTA_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void PORTA_IRQHandler() {
    NVIC_ClearPendingIRQ(PORTA_IRQn);

	//We don't actually have to test that it is
	//PTA4 since there is only one interrupt
	//configured, but this shows how to do it.
	if(PORTA->ISFR & (1 << SWITCH_PIN)) {
		//TODO
	}

    //Write a 1 to clear the ISFR bit.
    PORTA->ISFR |= (1 << SWITCH_PIN);

}

int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    PRINTF("Hello World\r\n");
    setTPMClock();
    initPWM();
	initButton();
    setPWM(RED, 0);
    setPWM(GREEN, 0);
    setPWM(BLUE, 0);
    startPWM();

    initADC();
    startADC(ADC_SE0);
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;

        // Sample way to convert values
        int red = (int) (result[0] / 4096.0 * 255.0);
        int green = (int) (result[1] / 4096.0 * 255.0);
        int blue = (red + green) / 2;

        PRINTF("RED = %d GREEN = %d BLUE = %d\r\n", red, green, blue);
        setPWM(RED, red);
        setPWM(GREEN, green);
        setPWM(BLUE, blue);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}

