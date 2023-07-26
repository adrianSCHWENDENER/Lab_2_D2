#include "LCD.h"
#include <xc.h>

#define _XTAL_FREQ 1000000

#define RS PORTDbits.RD6
#define EN PORTDbits.RD7
#define D0 PORTBbits.RB0
#define D1 PORTBbits.RB1
#define D2 PORTBbits.RB2
#define D3 PORTBbits.RB3
#define D4 PORTBbits.RB4
#define D5 PORTBbits.RB5
#define D6 PORTBbits.RB6
#define D7 PORTBbits.RB7

void Lcd_Port(char a) {
    if (a & 0b00000001)
        D0 = 1;
    else
        D0 = 0;
    
    if (a & 0b00000010)
        D1 = 1;
    else
        D1 = 0;
    
    if (a & 0b00000100)
        D2 = 1;
    else
        D2 = 0;
    
    if (a & 0b00001000)
        D3 = 1;
    else
        D3 = 0;
    
    if (a & 0b00010000)
        D4 = 1;
    else
        D4 = 0;

    if (a & 0b00100000)
        D5 = 1;
    else
        D5 = 0;

    if (a & 0b01000000)
        D6 = 1;
    else
        D6 = 0;

    if (a & 0b10000000)
        D7 = 1;
    else
        D7 = 0;
}

void Lcd_Cmd(char a) {
    RS = 0;
    Lcd_Port(a);
    EN = 1;
    __delay_ms(4);
    EN = 0;
}

void Lcd_Clear() {
    Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b) {
    char temp;
    if (a == 1) {
        temp = 0x80 + b - 1;
        Lcd_Cmd(temp);
    } else if (a == 2) {
        temp = 0xC0 + b - 1;
        Lcd_Cmd(temp);
    }
}

void Lcd_Init() {
    Lcd_Port(0x00);
    __delay_ms(20);
    Lcd_Cmd(0x03);
    __delay_ms(5);
    Lcd_Cmd(0x03);
    __delay_ms(11);
    Lcd_Cmd(0x03);
    /////////////////////////////////////////////////////
    //Iniciacion de 8 bits
    
    Lcd_Cmd(0b00111000);
    Lcd_Cmd(0b00001100);
    Lcd_Cmd(0b00000001);
    Lcd_Cmd(0b00000110);
    
    //Iniciacion de 4 bits
    /*
    Lcd_Cmd(0x02);
    Lcd_Cmd(0x02);
    Lcd_Cmd(0x08);
    Lcd_Cmd(0x00);
    Lcd_Cmd(0x0C);
    Lcd_Cmd(0x00);
    Lcd_Cmd(0x06);
    */
}

void Lcd_Write_Char(char a) {
    RS = 1; // => RS = 1
    Lcd_Port(a); //Data transfer
    EN = 1;
    __delay_us(40);
    EN = 0;
}

void Lcd_Write_String(char *a) {
    int i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right() {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left() {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x08);
}