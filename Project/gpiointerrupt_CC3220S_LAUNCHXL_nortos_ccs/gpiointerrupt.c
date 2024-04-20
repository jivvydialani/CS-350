/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART_write(uart, &output, x);

// I2C Global Variables and UART Global Variables
char output[64];
int bytesToSend;

static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}
sensors[3] = {
              { 0x48, 0x0000, "11X" },
              { 0x49, 0x0000, "116" },
              { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;


// Driver Handles - Global variables
UART_Handle uart;
I2C_Handle i2c;
Timer_Handle timer0;

volatile unsigned char TimerFlag = 0;

int16_t temperature = 0;
int setPoint = 25;
unsigned char heat = 0;
int seconds = 0;

void initUART(void) {
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

// initUART() needs to be called before this function.
void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;
    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {

        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }
    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses
    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;
    found = false;

    for (i=0; i<3; ++i) {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}


int16_t readTempSensor(void) {

    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction)) {
        /*
         * Extract degrees C from the received data;
         * see TMP sensor datasheet
         */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
         * If the MSB is set '1', then we have a 2's complement
         * negative value which needs to be sign extended
         */
        if (rxBuffer[0] & 0x80) {
            temperature |= 0xF000;
        }
    }
    else {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}


void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}


void initTimer(void){
    //Timer_Handle timer0;
    Timer_Params params;

    // Init the driver
    Timer_init();
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

// Button States
enum BUTTON_STATES {BUTTON_INIT, BUTTON1, BUTTON2} BUTTON_STATE;

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    // Button 1 pressed
    BUTTON_STATE = BUTTON1;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // Button 2 pressed
    BUTTON_STATE = BUTTON2;
}


void Read_Button() {
    switch (BUTTON_STATE) {
        case BUTTON_INIT:
            break;
        case BUTTON1:
            setPoint += 1;
            BUTTON_STATE = BUTTON_INIT;
            break;
        case BUTTON2:
            setPoint -= 1;
            BUTTON_STATE = BUTTON_INIT;
            break;
    }
}

// Temperature and LED States
enum TEMP_STATES {TEMP_INIT, TEMP_READ} TEMP_STATE;
enum LED_STATES {LED_INIT, LED_ON, LED_OFF} LED_STATE;


void Read_Temp() {
    switch (TEMP_STATE) {
        case TEMP_INIT:
            TEMP_STATE = TEMP_READ;
            break;
        case TEMP_READ:
            temperature = readTempSensor();
            if (temperature < setPoint) {
                LED_STATE = LED_ON;
            }
            if (temperature >= setPoint) {
                LED_STATE = LED_OFF;
            }
            break;
    }
}

// Called every 500ms
void Read_LED() {
    switch (LED_STATE) {
        case LED_INIT:
            break;
        case LED_ON:
            heat = 1;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
            break;
        case LED_OFF:
            heat = 0;
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
            break;
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    // Initialize time to display output right away
    unsigned long buttonTime = 200;
    unsigned long tempTime = 500;
    unsigned long updateTime= 1000;

    const unsigned long timerIncrement = 100; // Timers increment every 100ms

    /* Call driver init functions */
    GPIO_init();
    initTimer();
    initUART();
    initI2C();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn off user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    // Initialize state machines
    BUTTON_STATE = BUTTON_INIT;
    TEMP_STATE = TEMP_INIT;
    LED_STATE = LED_INIT;

    while(1){
        if (buttonTime >= 200) { // Read button every 200ms
            Read_Button();
            buttonTime = 0;
        }
        if (tempTime >= 500) { // Read temperature every 500ms
            Read_Temp();
            Read_LED();
            tempTime = 0;
        }
        if (updateTime >= 1000) {
            if (temperature == 0){
                DISPLAY(snprintf(output, 64, "<Retrieving Data>\n\r", temperature, setPoint, heat, seconds))
            }
            else { // After sensor retrieves temp data, displays current data
                DISPLAY(snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", temperature, setPoint, heat, seconds))
            }
            seconds += 1;
            updateTime = 0;
        }

        while(!TimerFlag){}
        /* Set the timer flag variable to FALSE */
        TimerFlag = 0;

        // Increment timers every 100ms
        updateTime += timerIncrement;
        tempTime += timerIncrement;
        buttonTime += timerIncrement;
    }
}
