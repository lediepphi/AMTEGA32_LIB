
#include "adc.h" 

void initADC(){
  #define AREF_MODE		(0<<REFS1)|(0<<REFS0)		// Voltage on AREF pin as reference voltage
  #define AVCC_MODE		(0<<REFS1)|(1<<REFS0)		// Voltage on AVCC pin as reference voltage
  #define INT_MODE		(1<<REFS1)|(1<<REFS0)		// Internal reference voltage 2.56V
  
  #define ADC_VREF_TYPE		AVCC_MODE				// select AREF pin as reference voltage
 
 //ADCSRA (ADC Control and Status RegisterA ) ***
 //ADEN (ADC Enable by writing this bit 1 to enables the ADC. By writing it to 0 the ADC is turned off)
 // ADPS2 , ADPS1 , ADPS0 (ADC Prescaler Select Bits) 
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);	
  
  //ADMUX (ADC Multiplexer Selection Register)	***	
  ADMUX = ADC_VREF_TYPE;
}


uint16_t ReadADC(unsigned char adc_channel){
   ADMUX = adc_channel|ADC_VREF_TYPE;			// adc_channel (0->7 \ : 0000 -> 0111) 
   
   //ADSC: ADC Start Conversion
   ADCSRA |= (1<<ADSC);						//ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADSC)
   
   //ADIF: ADC Interrupt Flag. This bit is set when an ADC conversion completes and the Data Registers are updated
   loop_until_bit_is_set(ADCSRA,ADIF);		//loop until the ADIF bit in the ADCSRA register is set to 1
   
   //ADCW: ADC Word (combination ADCL and ADCH – The ADC Data Register)
   return ADCW;
}