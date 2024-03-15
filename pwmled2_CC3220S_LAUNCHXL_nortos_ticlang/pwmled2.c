/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 * Assignment:  Week2 Milestone
 * Date:    1/16/2024
 *
 */


/*
 *  ======== pwmled2.c ========
 */
/* For usleep() */
#include <unistd.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/PWM.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/*
 *  ======== mainThread ========
 *  Task periodically increments the PWM duty for the on board LED.
 */
void *mainThread(void *arg0)
{
    /* Period and duty in microseconds */
    uint16_t pwmPeriod = 3000;
    uint16_t duty      = 0;
    uint16_t dutyInc   = 100;

    /* Sleep time in microseconds */
    uint32_t time   = 50000; // use this for .5 second interval
    uint32_t time2 = 1000000; // using this for 1 second interval (used for sleep timer)
    PWM_Handle pwm1 = NULL;
    PWM_Handle pwm2 = NULL;
    PWM_Params params;

    /* Call driver init functions. */
    PWM_init();


    /* Note: duty units can also be in FRACTION, in US mode it is microseconds */
    PWM_Params_init(&params);
    params.dutyUnits   = PWM_DUTY_US;
    params.dutyValue   = 0;
    params.periodUnits = PWM_PERIOD_US;
    params.periodValue = pwmPeriod;
    pwm1               = PWM_open(CONFIG_PWM_0, &params);
    if (pwm1 == NULL)
    {
        /* CONFIG_PWM_0 did not open */
        while (1) {}
    }

    PWM_start(pwm1);

    pwm2 = PWM_open(CONFIG_PWM_1, &params);
    if (pwm2 == NULL)
    {
        /* CONFIG_PWM_0 did not open */
        while (1) {}
    }

    PWM_start(pwm2);

    /* creating variables to get dutyValue at 90% and 10% of the period
     * Note that at 0% the output is always low, at 100% the output is always high */
    uint32_t dutyValue90 = pwmPeriod * .9;
    uint32_t dutyValue10 = pwmPeriod * .1;

    // Use these if we want to set duty units to FRACTIONAL
//    uint32_t dutyValue90 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 90) / 100);
//    uint32_t dutyValue10 = (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * 10) / 100);

    /* Perform instructions inside the while loop forever */
    while (1)
    {

        /* Call the set duty function on each PWM at the correct percentage values
         * 90% for PWM1, and 10% for PWM2
         * then sleep for 1sec
         * then set duty function on each PWM at correct percentage value
         * 0% for PWM1, and 90% for PWM2
         * then sleep for 1sec
         */
        PWM_setDuty(pwm1, dutyValue90);
        PWM_setDuty(pwm2, dutyValue10);
        usleep(time2);
        PWM_setDuty(pwm1, duty);
        PWM_setDuty(pwm2, dutyValue90);
        usleep(time2);


        // keep this is code from pwmled2 example that fades LEDs in and out at 50 millisecond interval for future reference
//        PWM_setDuty(pwm1, duty);
//        PWM_setDuty(pwm2, duty);
//        duty = (duty + dutyInc);
//
//        if (duty == pwmPeriod || (!duty))
//        {
//            dutyInc = -dutyInc;
//        }
//
//        usleep(time);
    }
}
