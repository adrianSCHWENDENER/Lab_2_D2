#include "ADC_int.h"

void setup_ADC(int channel);
int read_ADC();
void change_channel_ADC(int channel);
int get_channel_ADC();

void setup_ADC(int channel){
    ADCON0 = 0B01000001;                                    // Fosc/8 (2us), AnCH0, ADC encendido 
    ADCON1 = 0B00000000;                                    // Voltajes de referencia y formato a la izquierda
    ANSEL = 0;
    ANSELH = 0;
            
    switch (channel){
        case 0:
            ANSELbits.ANS0 = 1;
            TRISAbits.TRISA0 = 1;
            break;
        case 1:
            ADCON0bits.CHS = 0B0001;
            ANSELbits.ANS1 = 1;
            TRISAbits.TRISA1 = 1;
            break;
        case 2:
            ADCON0bits.CHS = 0B0010;
            ANSELbits.ANS2 = 1;
            TRISAbits.TRISA2 = 1;
            break;
        case 3:
            ADCON0bits.CHS = 0B0011;
            ANSELbits.ANS3 = 1;
            TRISAbits.TRISA3 = 1;
            break;
        case 4:
            ADCON0bits.CHS = 0B0100;
            ANSELbits.ANS4 = 1;
            TRISAbits.TRISA5 = 1;
            break;
        case 5:
            ADCON0bits.CHS = 0B0101;
            ANSELbits.ANS5 = 1;
            TRISEbits.TRISE0 = 1;
            break;
        case 6:
            ADCON0bits.CHS = 0B0110;
            ANSELbits.ANS6 = 1;
            TRISEbits.TRISE1 = 1;
            break;
        case 7:
            ADCON0bits.CHS = 0B0111;
            ANSELbits.ANS7 = 1;
            TRISEbits.TRISE2 = 1;
            break;
        case 8:
            ADCON0bits.CHS = 0B1000;
            ANSELHbits.ANS8 = 1;
            TRISBbits.TRISB2 = 1;
            break;
        case 9:
            ADCON0bits.CHS = 0B1001;
            ANSELHbits.ANS9 = 1;
            TRISBbits.TRISB3 = 1;
            break;
        case 10:
            ADCON0bits.CHS = 0B1010;
            ANSELHbits.ANS10 = 1;
            TRISBbits.TRISB1 = 1;
            break;
        case 11:
            ADCON0bits.CHS = 0B1011;
            ANSELHbits.ANS11 = 1;
            TRISBbits.TRISB4 = 1;
            break;
        case 12:
            ADCON0bits.CHS = 0B1100;
            ANSELHbits.ANS12 = 1;
            TRISBbits.TRISB0 = 1;
            break;
        case 13:
            ADCON0bits.CHS = 0B1101;
            ANSELHbits.ANS13 = 1;
            TRISBbits.TRISB5 = 1;
            break;            
    }
}

int read_ADC(){
    return ADRESH;
}

void change_channel_ADC(int channel){
    switch (channel){
        case 0:
            ADCON0bits.CHS = 0B0001;
            ANSELbits.ANS1 = 1;
            break;
        case 1:
            ADCON0bits.CHS = 0B0010;
            ANSELbits.ANS2 = 1;
            break;
        case 2:
            ADCON0bits.CHS = 0B0011;
            ANSELbits.ANS3 = 1;
            break;
        case 3:
            ADCON0bits.CHS = 0B0100;
            ANSELbits.ANS4 = 1;
            break;
        case 4:
            ADCON0bits.CHS = 0B0101;
            ANSELbits.ANS5 = 1;
            break;
        case 5:
            ADCON0bits.CHS = 0B0110;
            ANSELbits.ANS6 = 1;
            break;
        case 6:
            ADCON0bits.CHS = 0B0111;
            ANSELbits.ANS7 = 1;
            break;
        case 7:
            ADCON0bits.CHS = 0B1000;
            ANSELHbits.ANS8 = 1;
            break;
        case 8:
            ADCON0bits.CHS = 0B1001;
            ANSELHbits.ANS9 = 1;
            break;
        case 9:
            ADCON0bits.CHS = 0B1010;
            ANSELHbits.ANS10 = 1;
            break;
        case 10:
            ADCON0bits.CHS = 0B1011;
            ANSELHbits.ANS11 = 1;
            break;
        case 11:
            ADCON0bits.CHS = 0B1100;
            ANSELHbits.ANS12 = 1;
            break;
        case 12:
            ADCON0bits.CHS = 0B1101;
            ANSELHbits.ANS13 = 1;
            break;
        case 13:
            ADCON0bits.CHS = 0B0000;
            ANSELbits.ANS0 = 1;
            break;           
    }
}

int get_channel_ADC(){
    if (ADCON0bits.CHS == 0B0000){
        return 0;
    }
    else if (ADCON0bits.CHS == 0B0001){
        return 1;
    }
    else if (ADCON0bits.CHS == 0B0010){
        return 2;
    }
    else if (ADCON0bits.CHS == 0B0011){
        return 3;
    }
    else if (ADCON0bits.CHS == 0B0100){
        return 4;
    }
    else if (ADCON0bits.CHS == 0B0101){
        return 5;
    }
    else if (ADCON0bits.CHS == 0B0110){
        return 6;
    }
    else if (ADCON0bits.CHS == 0B0111){
        return 7;
    }
    else if (ADCON0bits.CHS == 0B1000){
        return 8;
    }
    else if (ADCON0bits.CHS == 0B1001){
        return 9;
    }
    else if (ADCON0bits.CHS == 0B1010){
        return 10;
    }
    else if (ADCON0bits.CHS == 0B1011){
        return 11;
    }
    else if (ADCON0bits.CHS == 0B1100){
        return 12;
    }
    else if (ADCON0bits.CHS == 0B1101){
        return 13;
    }

}