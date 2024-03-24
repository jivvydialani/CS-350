/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



/*
 * Author:  Chris Trimmer
 * Course:  CS350 Emerging Systems Architecture & Technologies
 * Assignment:  Week3 Milestone 2
 * Date:    1/24/2024
 *
 */


/*
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>


// new includes
#include <stdio.h>
#include <ctype.h>


/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/* Enum used for States (RL = RedLight)
 * SMStart is the initial state
 * ON/OFF are desired states for turning light on and off
 * L1 and L2 represent Letters that we use while waiting for the next char
 *      we need these because we only read one letter at a time, and have to test
 *      next input
 */
enum RL_States { SMStart, L1, L2, ON, OFF } rlState;


/* SMTick function implements the State Machine every tick
 * Upon receiving input from UART, the SM starts
 */
void SMTick(char input) {

    // determine if we need to transition based on input
    switch (rlState) {
        case SMStart:
            if (input == 'O') {
                rlState = L1;
            }
            else {
                rlState = SMStart;
            }
            break;

        case L1:
            if (input == 'N') {
                rlState = ON;
            }
            else if (input == 'F') {
                rlState = L2;
            }
            else {
                rlState = SMStart;
            }
            break;

        case L2:
            if (input == 'F') {
                rlState = OFF;
            }
            else {
                rlState = SMStart;
            }
            break;

        default:
            rlState = SMStart;
            break;
    }


    /* determine action to take based on the current/new state
     * set state back to SMStart after taking action
     */
    switch (rlState) {
        case ON:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON); // turn led one
            rlState = SMStart;
            break;

        case OFF:
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF); // turn led off
            rlState = SMStart;
            break;

        // don't need to do anything here if we are not in ON or OFF
        default:
            break;
    }

}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{

    char input;
    const char echoPrompt[] = "Echoing characters chris trimmer week3:\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;
    uint32_t status     = UART2_STATUS_SUCCESS;

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }


    // function to write the intro line above to UART before we start the state machine
    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesWritten);

    // red light state begins at SMStart
    rlState = SMStart;

    /* Loop forever echoing */
    while (1)
    {

        // implementation to read from UART2 every tick
        bytesRead = 0;
        while (bytesRead == 0)
        {
            // attempt to read 1-byte from input
            status = UART2_read(uart, &input, 1, &bytesRead);

            if (status != UART2_STATUS_SUCCESS)
            {
                /* UART2_read() failed */
                while (1) {}
            }



            // if we read input, then capitalize it in case input is lowercase
            input = toupper(input);

            // call the tick function to run through the state machine with the input
            SMTick(input);

        }


        // implementation to write the output (this will be the char that was just read)
        bytesWritten = 0;
        while (bytesWritten == 0)
        {
            // write the char to UART
            status = UART2_write(uart, &input, 1, &bytesWritten);

            if (status != UART2_STATUS_SUCCESS)
            {
                /* UART2_write() failed */
                while (1) {}
            }
        }
    }
}

