
#ifndef INCFILE1_H_
#define INCFILE1_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "util/delay.h"

void initADC();
uint16_t ReadADC(unsigned char adc_channel);

#endif /* INCFILE1_H_ */