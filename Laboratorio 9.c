/* 
 * File:   LAB9.c
 * Author: ANGELA
 *
 * Created on 21 de abril de 2023, 12:58
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdint.h>

//definicion de frecuencia para delay
#define _XTAL_FREQ 4000000 
int APAGADO = 0;

//PROTOTIPO DE FUNCIONES
void setup(void);
uint8_t read_EEPROM(uint8_t ADRESS);
void write_EEPROM(uint8_t ADRESS, uint8_t data);

//INTERRUPCIONES
void __interrupt() isr(void){
    if(INTCONbits.RBIF){
        if (!PORTBbits.RB0){
            APAGADO = 1;
            PORTEbits.RE0 = 1;
            SLEEP();
        }
        else if (PORTBbits.RB1 == 0 && APAGADO == 1){
            APAGADO = 0;
            PORTEbits.RE0 = 0;
            PORTD = read_EEPROM(0);
        }
        else if(!PORTBbits.RB2){
            write_EEPROM(0, ADRESH);    
        }
        INTCONbits.RBIF = 0;
    }
    if(PIR1bits.ADIF){
        if(ADCON0bits.CHS == 0){
            PORTC = ADRESH;
        }
        PIR1bits.ADIF = 0;
    }
    return;
} 

void main(void){
    setup();
    while(1){
        if(!APAGADO){
            if(ADCON0bits.GO == 0){
                ADCON0bits.GO = 1;
            }
        }
    }
    return;
}

void setup(void){
    ANSEL = 0b00000001;
    ANSELH = 0;
    
    TRISA = 0b00000001;
    PORTA = 0x00;
    
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    
    TRISC = 0b00000000;
    PORTC = 0x00;
    TRISD = 0b00000000;
    PORTD = 0x00; 
    TRISE = 0b00000000;
    PORTE = 0x00;
    
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    
    //CONFIGURACION DE OSCILADOS
    OSCCONbits.IRCF = 0b0110;
    OSCCONbits.SCS = 1;
    
    //CONFIGURACION DE INTERRUPCIONES
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    INTCONbits.RBIE = 1;
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    
    //CONFIGURACION ADC
    ADCON0bits.ADCS = 0b11;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    ADCON0bits.CHS = 0b0000;
    ADCON1bits.ADFM = 0;
    ADCON0bits.ADON = 1;
    __delay_us(40);
    
    return; 
} 

uint8_t read_EEPROM(uint8_t ADRESS){
    EEADR = ADRESS;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    return EEDAT;
}

void write_EEPROM(uint8_t ADRESS, uint8_t data){
    EEADR = ADRESS;
    EEDAT = data;
    EECON1bits.EEPGD = 0;
    EECON1bits.WREN = 1;
    
    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xAA;
    
    
    EECON1bits.WR = 1;
    EECON1bits.WREN = 0;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;
    
}
