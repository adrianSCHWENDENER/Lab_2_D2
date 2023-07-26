/*
 * File:   Principal.c
 * Author: schwe
 *
 * Created on 19 de julio de 2023, 03:55 PM
 */

//CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT                        //Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF                                   //Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF                                  //Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF                                  //RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF                                     //Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF                                    //Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF                                  //Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF                                   //Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF                                  //Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF                                    //Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V                               //Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF                                    //Flash Program Memory Self Write Enable bits (Write protection off)

//Librerías
#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include "ADC_int.h"
#include "LCD.h"
#include "UART.h"

//Definición de variables
#define _XTAL_FREQ 1000000
int ADC;
uint8_t indicador = 0B00110000;
int condicion;
uint8_t contador;
int esperar;
char cADC[2];
char ncADC[4];
char charcontador[2];

//Prototipos
void setup(void);
void main(void);
int mapear(int valor, int min, int max, int nmin, int nmax);

//Setup General
void setup(void){
    //Oscilador
    OSCCON = 0B01000001;                                    //Oscilador a 1Mhz
    
    //Interrupciones
    INTCON = 0B11000000;                                    //Int globales, PIE1 activadas
    PIE1 = 0B01100000;                                      //Int ADC, UART activadas
    PIR1 = 0B00000000;
    OPTION_REG = 0;
    
    //ADC
    setup_ADC(0);
    
    //UART
    UART_config(9600, 1000000, BOTH);

    //Entradas y salidas
    TRISB = 0;
    //TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    
    //Valores iniciales de variables y puertos
    PORTA = 0;
    PORTB = 0;
    //PORTC = 0;
    PORTD = 0;
    PORTE = 0;
    
    esperar = 0;
    contador = 0;
    ADC = 0;
    
    return;
}

//Interrupcion
void __interrupt() isr(void) {
    if (PIR1bits.ADIF){
        ADC = read_ADC();
        ADC = mapear(ADC, 0, 255, 0, 500);
        sprintf(cADC, "%d", ADC);                           //Convertir ADC en tipo char
    }
    
    if (PIR1bits.RCIF){                                     //Si se puede recibir valor de UART
        indicador = RCREG;                                  //Guardar valor en indicador
        if (indicador == 0B00110010){                       //Si indicador es 2
            while (esperar == 0){                           //Esperar a que se presione una tecla
                condicion = RCREG;
                if (condicion == 0B00101011){
                    contador ++;
                    esperar = 1;
                }
                else if (condicion == 0B00101101){
                    contador --;
                    esperar = 1;
                }
            }
            indicador = 0B00110000;                         //Regresar al menú
            esperar = 0;                                    //Resear variable
        }
    }
    
    PIR1bits.RCIF = 0;
    PIR1bits.ADIF = 0;                                      //Limpia la bandera de interrupción
    return;
}

//Loop
void main(void) {
    setup();
    Lcd_Init();
    Lcd_Clear();
    while(1){    
        if (ADCON0bits.GO == 0){
            __delay_ms(5);
            ADCON0bits.GO = 1;
        }
        
        if (indicador == 0B00110000){                       //Si esta activado el menú
            UART_write_string("1. Leer potenciometro ");    //Mostrar
            UART_write_string("2. Enviar valor ");
            indicador = 0;                                  //Evitar repetir menú y esperar indicador
        }
        else if (indicador == 0B00110001){                  //Si indicador es 1
            Lcd_Set_Cursor(2,1);                            //Mostrar valor del potenciometro
            if(ADC < 10){
                cADC[2] = cADC[0];
                cADC[1] = '0';
                cADC[0] = '0';
            }
            else if(ADC < 100){
                cADC[2] = cADC[1];
                cADC[1] = cADC[0];
                cADC[0] = '0';
            }
            ncADC[0] = cADC[0];
            ncADC[1] = '.';
            ncADC[2] = cADC[1];
            ncADC[3] = cADC[2];
            ncADC[4] = 0;
            
            Lcd_Write_String(ncADC);
            
            indicador = 0B00110000;                         //Regresar al menú
        }

        Lcd_Set_Cursor(1, 1);
        Lcd_Write_String("POT1");
        Lcd_Set_Cursor(1, 7);
        Lcd_Write_String("Contador");
        Lcd_Set_Cursor(2,7);
        sprintf(charcontador, "%d", contador);
        if(contador < 10){
            charcontador[2] = charcontador[0];
            charcontador[1] = '0';
            charcontador[0] = '0';
        }
        else if(contador < 100){
            charcontador[2] = charcontador[1];
            charcontador[1] = charcontador[0];
            charcontador[0] = '0';
        }
        Lcd_Write_String(charcontador);
    }
}

//Funcioness
int mapear (int valor, int min, int max, int nmin, int nmax){
    int nvalor = nmin + (valor - min) * (long)(nmax - nmin) / (max - min);
    return nvalor;
}
