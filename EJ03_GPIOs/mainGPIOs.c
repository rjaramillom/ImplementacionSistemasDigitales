/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>


//void LED_on(void){
//    P2->OUT |= 0X07;       // Encendido de LEDs, 2.0, 2.1 y 2.2 en estado alto
//}
//void LED_off(void){
//    P2->OUT &= ~0X07;      // Apagado de LEDs, 2.0, 2.1 y 2.2 en estado bajo
//}
//void LED_Toggle(void){
//    P2->OUT ^= 0X07;       // Toggle de LEDs, 2.0, 2.1 y 2.2 en estado alto
//}

void LED_Init(void);

int contador = 0x00;

int main(void)
{

    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();
    /* Configuración de GPIOs */

    LED_Init();
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);      // P1.4 -> Entrada con Pull-up
    P2->OUT = contador;                                                 // GPIOs -> 0
    int i;
    while(1)
    {
        if (!(P1IN & BIT4)){
            // Por defecto la velocidad de trabajo es de 3 MHz
            // Retardo de 15 milisegundos 15*3000000/1000
            for(i = 0;i<45000;i++){
            }
            /*Retardo con cilcos de reloj */
            //__delay_cycles(45000);

            // Accion para P1.4 = 0
            contador = contador + 1;
            P2->OUT = contador;
            if (contador >= 8){
                 contador = 0;
             }

        }else{
            // Accion para P1.1 = 1
        }
    }
}

void LED_Init(void){
    P2->SEL0 &= ~0X07;       // Configuración GPIOs 2.0, 2.1, 2.2
    P2->SEL1 &= ~0X07;
    P2->DIR |= 0X07;        // 2.0, 2.1, 2.2 como salida
}






