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

/*
 * Milestone 2
 * Scott Wood and David Sheppard
 *
 */


#include <msp430F5529.h>
#include <math.h>

float realTemp;
float desiredTemp = 25; //initialization default
float adcReading;
int adcReady = 1;

void setPWM();

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  REFCTL0 &= ~REFMSTR;                      // Reset REFMSTR to hand over control to
                                            // ADC12_A ref control registers

  //bit 2.7 will be PWM output***************************************************************
  P2SEL &= ~BIT7;                           //set 2.7 to be GPIO
  P2DIR |= BIT7;                            //set 2.7 to be output

  //port 6.0 is analog input 0***************************************************************
  P6DIR &= ~BIT0;                           //set 6.0 to be input
  P6SEL |= BIT0;                            //set 6.0 to be A0 (input of A to D)

  //UART Setup*******************************************************************************
  P3SEL |= BIT3 + BIT4;                     //enable UART for these pins
  UCA1CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
  // Use Table 24-5 in Family User Guide for BAUD rate calculation
  UCA1BR0 = 104;                            // sets baud rate to 9600 (16000000/16/9600)
  UCA1BR1 = 0x00;
  UCA1MCTL |= UCOS16 | UCBRF_3 | UCBRS_0;
  UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
  UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
  UCA1TXBUF = 0;                            //set RX buffer to 0 for testing purposes

  //PWM stuff*********************************************************************************
  TA1CCTL1 = CCIE;                          // CCR1 interrupt enabled for TA1
  TA1CCR0 = 262;                            //Set the period in the Timer A Capture/Compare 0 register to 1000 us.
  //TA1CCTL1 = OUTMOD_7;
  TA1CCR1 = 131;                            //The initial period in microseconds that the power is ON. It's half the time, which translates to a 50% duty cycle.
  TA1CTL = TASSEL_2 + MC_1 + ID_2 + TAIE;   //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.

  ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
  ADC12CTL1 = ADC12SHP;                     // Use sampling timer
  ADC12IE = 0x01;                           // Enable interrupt
  ADC12CTL0 |= ADC12ENC;                    //Enable conversion

  //Timer Setup for Software PWM**************************************************************
  TA1CCTL1 = CCIE;
  TA1CCR0 = 262;                            //Set the period in the Timer A Capture/Compare 0 register to 1000 us.
  TA1CCR1 = 131;                            //The initial period in microseconds that the power is ON. It's half the time, which translates to a 50% duty cycle.
  TA1CTL = TASSEL_2 + MC_1 + ID_2 + TACLR + TAIE;   //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.

  __bis_SR_register(/*LPMO + */GIE);        // LPM0 with interrupts enabled

  //polling for ADC values********************************************************************
  while(1)
  {
    ADC12CTL0 |= ADC12SC;                   // Sampling and conversion start
    //while(!(ADC12IFG & ADC12IFG0));       //wait for sample to be ready
    while(adcReady == 0){};                 //run this loop while waiting for ADC to finish conversion
    setPWM();                               //set PWM values to change duty cycle
    adcReady = 0;                           //adc no longer ready

  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    adcReady = 1;
    adcReading = ADC12MEM0;
  /*switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
    setPWM();

    /*__bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }*/

    __bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
}


void setPWM()
{

    //PTAT equation: Vo = 6.25 mV/C + 424 mV
    //therefore, Temp = (Vo - 0.424) / 0.00625

    float analogVoltage = adcReading * (3.3 / 4096);    //convert digital reading back to analog voltage value

    realTemp = ((analogVoltage - 0.424) / 0.00625);     //convert analog voltage to temperature

    while (!(UCA1IFG & UCTXIFG));                       //if TX buffer ready, send temp reading out thru TX
    UCA1TXBUF = realTemp;

    float difference = realTemp - desiredTemp;          //determine difference between real and desired temp

    float dc_increase = difference;                     //increasing/decreasing 1% DC for every degree C off

    if(difference > 2)                                  //if temp is too high
    TA1CCR1 += 262/dc_increase;                         //increase DC
    else if(difference < -2)                            //if temp is too low
        TA1CCR1 += 262/dc_increase;                     //decrease CCR1

    if(TA1CCR1 > 261)                                   //ensure CCR1 never gets about 261 (262 is CCR0)
        TA1CCR1 = 261;

}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)

{
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    /*while (!(UCA1IFG & UCTXIFG));             // USCI_A0 TX buffer ready?
    UCA1TXBUF = UCA1RXBUF;                  // TX -> RXed character
    break;*/
    desiredTemp = UCA1RXBUF;
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(TA1IV,14)) //testing timer interrupt vector
  {
    case  0: break;                          // No interrupt
    case  2: P2OUT &= ~BIT7;                 //if CCR1 is reached, set low
             break;
    case  4: break;                          // CCR2 not used
    case  6: break;                          // reserved
    case  8: break;                          // reserved
    case 10: break;                          // reserved
    case 12: break;                          // reserved
    case 14: P2OUT |= BIT7;                  // if CCR0 overflows, set high
             break;
    default: break;
  }
  TA1IV &= ~TA1IV_TA1IFG; // Clear the Timer interrupt Flag
}






