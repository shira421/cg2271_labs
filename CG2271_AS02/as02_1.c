/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    as02_1.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
/*insert other include files here. */

/*insert other definitions and declarations here. */

// LED pin numbers
#define RED_PIN		31	// PTE31
#define GREEN_PIN	5	// PTD5
#define BLUE_PIN	29	// PTE29

typedef enum tl {
	RED, GREEN, BLUE
} TLED;

//TODO
void setMCGIRCLK() {
	//clear CLKS
	MCG->C1 &= ~MCG_C1_CLKS_MASK;

	//set IRCLKEN to enable LIRC
	MCG->C1 |= ((MCG_C1_CLKS(0b01)| MCG_C1_IRCLKEN_MASK));

	//choose the 2MHz Clock
	MCG->C2 &= ~MCG_C2_IRCS_MASK;

	//set FRCDIV and LIRC_DIV2
	MCG->SC &= ~MCG_SC_FCRDIV_MASK;
	MCG->SC |= MCG_SC_FCRDIV(0b0);
	MCG->MC &= ~MCG_MC_LIRC_DIV2_MASK;
	MCG->MC |= MCG_MC_LIRC_DIV2(0b001);
}

void initTimer() {
	NVIC_DisableIRQ(TPM0_IRQn);
    setMCGIRClk();

    //Turn on clock gating for TPM0
    SIM->SCGC6|=SIM_SCGC6_TPM0_MASK;
    SIM->SOPT2&=~SIM_SOPT2_TPMSRC_MASK;

    //Clear TPM clock source and select MCGIRCLK
    SIM->SOPT2|=SIM_SOPT2_TPMSRC(0b11);

    //Turn off TPM0 and clear Prescale counter
    TPM0->SC&=~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);

    //Initialize counter to 0
    TPM0->CNT=0;

    //Set Prescale Counter
    TPM0->SC|= ((TPM_SC_TOIE_MASK) | TPM_SC_PS(0b011));

    //Initialize modulo
    TPM0->MOD = 62500;

    //Set priority to highest and enable IRQ
    NVIC_SetPriority(TPM0_IRQn, 0);

    NVIC_EnableIRQ(TPM0_IRQn);
    //setMCGIRClk(); //uncomment this after question 2
}

void startTimer() {
    TPM0->SC |= TPM_SC_CMOD(0b1);
}

void stopTimer() {
    TPM0->SC &= ~TPM_SC_CMOD_MASK;
}

//count needs to be volatile.
volatile int count = 0;
void TPM0_IRQHandler() {
    //clear pending IRQ
	NVIC_ClearPendingIRQ(TPM0_IRQr);

	//Check if TOF is set
	if(TPM0->STATUS & TPM_STATUS_TOF_MASK) {
		// IF TOF is set, 1. count = (count + 1) % 6
		count = (count + 1) % 6;

		// 2. reset counter
		TPM0->CNT = 0;

		// 3. Clear TOF bit
		TPM0->STATUS |= TPM_STATUS_TOF_MASK;
	}
}

void initGPIO()

void ledOn(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PCOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PCOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PCOR |= (1 << BLUE_PIN);
		break;
	}
}

void ledOff(TLED led) {
	switch(led) {
	case RED:
		GPIOE->PSOR |= (1 << RED_PIN);
		break;

	case GREEN:
		GPIOD->PSOR |= (1 << GREEN_PIN);
		break;

	case BLUE:
		GPIOE->PSOR |= (1 << BLUE_PIN);
		break;
	}
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

    initGPIO();
    initTimer();
    PRINTF("TIMER DEMO\r\n");
    startTimer();

    ledOff(RED);
    ledOff(GREEN);
    ledOff(BLUE);

    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
		switch(count) {
		case 0:
			ledOn(RED);
			break;
		case 1:
			ledOff(RED);
			break;
		case 2:
			ledOn(GREEN);
			break;
		case 3:
			ledOff(GREEN);
			break;
		case 4:
			ledOn(BLUE);
			break;
		case 5:
			ledOff(BLUE);
			break;
		default:
			count=0;
		}
    }
    return 0 ;
}
