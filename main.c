/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430F552x Demo - ADC12, Sample A10 Temp and Convert to oC and oF
//
//  Description: A single sample is made on A10 with reference to internal
//  1.5V Vref. Software sets ADC12SC to start sample and conversion - ADC12SC
//  automatically cleared at EOC. ADC12 internal oscillator times sample
//  and conversion. In Mainloop MSP430 waits in LPM4 to save power until
//  ADC10 conversion complete, ADC12_ISR will force exit from any LPMx in
//  Mainloop on reti.
//  ACLK = n/a, MCLK = SMCLK = default DCO ~ 1.045MHz, ADC12CLK = ADC12OSC
//
//  Uncalibrated temperature measured from device to devive will vary do to
//  slope and offset variance from device to device - please see datasheet.
//
//  NOTE:1.REFMSTR bit in REFCTL0 regsiter is reset to allow the ADC12_A reference
//    control regsiters handle the reference setting. Upon resetting the REFMSTR
//    bit, all the settings in REFCTL are 'dont care' and the legacy ADC12
//    control bits (ADC12REFON, ADC12REF2_5, ADC12TCOFF and ADC12REFOUT) control
//    the reference system.
//    2. Use the TLV calibrated temperature to measure temperature
//   (the TLV CALIBRATED DATA IS STORED IN THE INFORMATION SEGMENT, SEE DEVICE DATASHEET)
//
//                MSP430F552x
//             -----------------
//         /|\|              XIN|-
//          | |                 |
//          --|RST          XOUT|-
//            |                 |
//            |A10              |
//
//   F. Chen
//   Texas Instruments Inc.
//   Dec. 2012
//   Built with IAR Embedded Workbench Version: 5.51.1 & Code Composer Studio V5.2.1
//******************************************************************************

#include <msp430F5529.h>

float realTemp;
float desiredTemp = 10; //default to be a very low temp (for testing)

//unsigned int temp;
//volatile float temperatureDegC;
//volatile float temperatureDegF;

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to
                                            // ADC12_A ref control registers
  //bit 4.1 will be PWM output
  P4SEL &= ~BIT1;   //set 4.0 to be GPIO
  P4DIR |= BIT1;    //set 4.0 to be output

  //port 6.0 is analog input 0
  P6DIR &= ~BIT0;   //set 6.0 to be input
  P6SEL |= BIT0;    //set 6.0 to be A0 (input of A to D)

  ADC12IE |= BIT0;
  ADC12MCTL0 = ADC12INCH_0;
  ADC12CTL0 |= ADC12ENC;
  ADC12MCTL0 |= ADC12SREF_0;



  ADC12CTL0 = ADC12SHT0_8 + ADC12REFON + ADC12ON;
                                            // Internal ref = 1.5V
  ADC12CTL1 = ADC12SHP;                     // enable sample timer
  //ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_10;  // ADC i/p ch A10 = temp sense i/p
  __delay_cycles(100);                       // delay to allow Ref to settle


  //Hardware PWM stuff
  //TA0CCTL0 = CCIE;                        // CCR0 interrupt enabled for TA0
  TA1CCTL1 = CCIE;
  TA1CCR0 = 262;                            //Set the period in the Timer A Capture/Compare 0 register to 1000 us.
  TA1CCTL1 = OUTMOD_7;
  TA1CCR1 = 131; //The initial period in microseconds that the power is ON. It's half the time, which translates to a 50% duty cycle.
  TA0CTL = TASSEL_2 + MC_1 + ID_2 + TAIE; //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.
  //__bis_SR_register(LPM0_bits); //Switch to low power mode 0.

  while(1)
  {
    //ADC12CTL0 &= ~ADC12SC;
    ADC12CTL0 |= ADC12SC;                   // Sampling and conversion start

    //Vo = 6.25 mV/C + 424 mV


    __bis_SR_register(LPM0_bits + GIE);     // LPM0 with interrupts enabled
    __no_operation();

    //__no_operation();                       // SET BREAKPOINT HERE
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void ADC12ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12ISR (void)
#else
#error Compiler not supported!
#endif
{

    realTemp = ((ADC12MEM0 - 0.424) / 0.00625);

    float difference = realTemp - desiredTemp;

    float dc_increase = difference;  //increasing/decreasing 1% DC for every degrees C off

    if(difference > 2)
        TA1CCR1 += 262/dc_increase;
    else if(difference < -2)
        TA1CCR1 += 262/dc_increase;


}


#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void){

    desiredTemp = UCA0RXBUF;

}

#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_A1 (void)
{

    //When CCR1 overflows, set low
    P4OUT &= ~BIT1;

}

#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMERA1_CCR0(void){

    P4OUT |= BIT1;  //set high when CCR0 overflows
}






