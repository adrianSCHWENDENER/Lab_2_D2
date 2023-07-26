/* 
 * File: UART
 * Author: schwe
 * Comments: Uso para configuracion y uso de UART
 * Revision history: 
 */

#ifndef UART_H
#define	UART_H
#include <stdio.h>

#include <stdio.h>
#include <stdint.h>
#include <xc.h> // include processor files - each processor file is guarded.  

typedef enum{
    RX, //0
    TX, //1
    BOTH //2
} CONECTION;

void UART_config(uint16_t baudrate, long fosc, CONECTION conection);
void UART_write_char(char a);
char UART_read_char();
void UART_write_string(char txt[]);

#endif	/* UART_H */

