/* --COPYRIGHT--,BSD
 * Copyright (c) 2016, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//*****************************************************************************
// iqmathlib_ex1_empty: IQmathLib empty project example.
//
// This example provides a template for new IQmathLib projects.
//
// B. Peterson
// Texas Instruments Inc.
// Febrero 18
//*****************************************************************************
#include <ti/devices/msp432p4xx/inc/msp432.h>

#include <stdint.h>

/* Select the global Q value */
#define GLOBAL_Q    12
// #define LED01 BIT0      /*Ob00000001*/
// #define PUSH01 BIT1     /*Ob00000010*/
#define PRESSED 0


/* Include the iqmathlib header files */
#include <ti/iqmathlib/QmathLib.h>
#include <ti/iqmathlib/IQmathLib.h>

int main(void)
{
    /* Disable WDT. */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /* Inicializar GPIOs
     * P1.0 -> Salida
     * P1.1 /> Entrada
     * */

    P1DIR = 0b00000001;
    //P1DIR = LED_01;
    //P1DIR = ~PUSH01;

    /* Configuracion de P1.1 con resistencias pullup */
    P1REN = 0b00000010;
    P1OUT = 0b00000010;

    while (1) {
        if ( !(P1IN & BIT1) ){
            P1OUT |= BIT0;
        }
        else
            P1OUT &= ~BIT0;
    }
}
