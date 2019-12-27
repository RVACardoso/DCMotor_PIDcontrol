#define FCY 29491200L

/* Include List */
#include <libpic30.h>
#include <p30F4011.h>
#include <stdio.h>
#include <uart.h>
#include <ports.h>
#include "xc.h"

/* For XC16 Ver >= 1.24 */
#pragma config FCKSMEN=CSW_FSCM_OFF
#pragma config FOS=PRI
#pragma config FPR=XT_PLL16 
#pragma config WDT=WDT_OFF
#pragma config MCLRE=MCLR_EN
#pragma config FPWRT=PWRT_OFF

//#define kp 0.0034
#define kp 0.006
//#define Ti 572723.
#define Ti 1502723.
//#define Td 143181.
#define Td 2000181.


//Global variables
unsigned int tempo=0, tempo_old=0;
int count=0, old_count=0, ic1_interr=0;
int duty=30;
float duty_var=0;
int error_0=0, error_1=0, error_2=0;
float sum=0.;

void UART_config(){
    //configure for UART2
    U2MODEbits.STSEL = 0;   //one stop bit
    U2MODEbits.PDSEL = 0;   //data length and parity: 8 bits, no parity
    U2BRG = 15;             //set baud rate 115200
    U2MODEbits.UARTEN = 1;  //enable uart 2
    U2STAbits.UTXEN = 1;    //enable uart 2 tx
    U2STAbits.URXISEL = 0;  //uart2 receive interrupt at each received character
    U2STAbits.UTXISEL = 0;  //uart2 send interrupt at each transmited character
    IFS1bits.U2RXIF = 0;    //uart2 receive interrupt clear flag
    IFS1bits.U2TXIF = 0;    //uart2 send interrupt clear flag
    IEC1bits.U2TXIE = 0;    //enable uart2 transmit interrupts
    IEC1bits.U2RXIE = 0;    //enable uart2 receive interrupts

    __C30_UART=2;           //point printf to UART2 (only needed if use printf)
}

void TIMER2_config(){
    /*Timer 2 10kHz*/
    T2CONbits.TON = 0;      //Timer_2 is OFF
    TMR2 = 0;               //resets Timer_2
    PR2 = 3000;            //sets the maximum count for Timer_2
    T2CONbits.TCS = 0;      //choose FCY as clock source for Timer_2
    T2CONbits.TCKPS = 0x00; //sets the Timer_2 pre-scaler to 1
    IFS0bits.T2IF = 0;      //clears Timer_2 interrupt flag
    _T2IE = 0;              //enables Timer_2 Interrupts 
}

void TIMER3_config(){
    T3CONbits.TON = 0;      //Timer_3 is OFF
    TMR3 = 0;               //resets Timer_3
    PR3 = 65000;            //sets the maximum count for Timer_3
    T3CONbits.TCS = 0;      //choose FCY as clock source for Timer_3
    T3CONbits.TCKPS = 0b01; //sets the Timer_3 pre-scaler to 1
    IFS0bits.T3IF = 0;      //clears Timer_3 interrupt flag
    _T3IE = 1;              //enables Timer_3 Interrupts
}

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt(void){
    IFS0bits.IC1IF = 0;
    tempo_old=tempo;
    tempo = TMR3;//IC7BUF;
    TMR3=0;   
    ic1_interr=1;
    old_count=count;
    count=0;
}

void IC1_config(){
    // Configuração do IC ativado em cada flanco descendente 
    _TRISD0 = 1;                  // define pin as input
    IC1CONbits.ICM= 0b000;         // Disable Input Capture module
    IC1CONbits.ICTMR= 0;         // Select Timer3 as the IC1 Time base
    IC1CONbits.ICI= 0b00;         // Interrupt on every capture event   
    IC1CONbits.ICM= 0b001;         // Generate capture event on every rising and falling 
    
    IFS0bits.IC1IF = 0;         // Clear IC1 Interrupt Status Flag
    IEC0bits.IC1IE = 1;         // Enable IC1 interrupt
   
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void){    
    IFS0bits.T3IF = 0;
    count++;

}   
void PID_control(){
    
    if(duty!=100 && duty!=20){
        if((sum<=-200 && error_0<0) || (sum>=200 && error_0>0)){
            //sum+=error_0;
        }else{
            sum+=error_0;
        }
    }
    duty_var = error_0+(74.*tempo/Ti)*sum+(Td/(74.*tempo))*(error_0-error_1);
    duty_var = 5.*kp*duty_var;
    int duty_temp = duty + duty_var;
    if (duty_temp < 20){
        duty = 20;
    }else if(duty_temp > 100){
        duty = 100;
    }else{
        duty=duty_temp;
    }
    OC2RS = 0.01*duty*PR2;
}

int main(void) {
        
    _TRISC13=0; 
    
    UART_config();
    TIMER2_config();
    TIMER3_config();
    
    /*configure for OC2 - 40kHz@50% */
    OC2RS = 0.01*duty*PR2;          //sets the initial duty_cycle
    OC2R = PR2/2;               //Initial Delay (only for the first cycle)
    OC2CONbits.OCM = 0b110;     //set OC2 desligado
    OC2CONbits.OCTSEL = 0;  //selects Timer_2 as the OC2 clock source
    
    IC1_config();
    
    T3CONbits.TON = 1;       //turn timer3 on
    T2CONbits.TON = 1;      //turns Timer_2 
    
    unsigned int setpoint=334; // 
    int joao = 0;
    int tempo_tmr=0, count_tmr=0;
    while(1){
        if(ic1_interr==1 && old_count==0){
            int freq = 1000000./tempo;
            error_2 = error_1;
            error_1 = error_0;
            error_0 = setpoint - freq;
            printf("\r\n TMR3: %u \t duty: %d \t freq: %u \t error: %d \t duty_var: %f \t sum: %f", tempo, duty, freq, error_0, duty_var, sum);
            PID_control();
            ic1_interr=0;
        }
    }
    return 0;
}
