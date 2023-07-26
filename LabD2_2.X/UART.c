#include "UART.h"
#include <stdint.h>
#include <stdio.h>

void UART_config(uint16_t baudrate, long fosc, CONECTION conection);
void UART_write_char(char a);
char UART_read_char();
void UART_write_string(char txt[]);

#define _XTAL_FREQ 1000000

void UART_config(uint16_t baudrate, long fosc, CONECTION conection){
    //uint16_t SPBRGtot;
    
    //SPBRGtot = (fosc/(baudrate*64))-1;
    //SPBRG = SPBRGtot;
    //SPBRGH = SPBRGtot >> 8;
    SPBRG = 25;
    SPBRGH = 0;
    BAUDCTL = 0B00001000;
    
    switch (conection){
        case RX:
            RCSTA = 0B10010000;
            TXSTA = 0B00000000;
            break;
        case TX:
            RCSTA = 0B00000000;
            TXSTA = 0B00100100;
            break;
        case BOTH:
            RCSTA = 0B10010000;
            TXSTA = 0B00100100;
            break;
    }
    return;
}

void UART_write_char(char a){
    if (PIR1bits.TXIF){
        TXREG = a;
        __delay_ms(1);
    }
    return;
}

void UART_write_string(char txt[]){
    for (uint8_t i = 0; txt[i] != '\0'; i++){               //Recorrer el string y mostrarlo en
        UART_write_char(txt[i]);
    }
}

char UART_read_char(){
    return RCREG;
}
