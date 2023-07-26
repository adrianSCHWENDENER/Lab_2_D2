 /* 
 * File: ADC_int.h
 * Author: schwe
 * Comments: Uso para interrupciones del ADC y canales
 * Revision history: 
 */

#ifndef ADC_int_H
#define	ADC_int_H

#include <pic16f887.h>
#include <xc.h> // include processor files - each processor file is guarded.  

void setup_ADC(int channel);
int read_ADC();
void change_channel_ADC(int channel);
int get_channel_ADC();

#endif	/* ADC_int_H */

