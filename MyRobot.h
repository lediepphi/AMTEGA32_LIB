#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <string.h>

#include "UART.h"
#include "I2C_LCD.h"
//#include "GM65.h"

#define C1A		PD2
#define C2B		PD3
#define C2A		PA0
#define C1B		PA1


#define EB		PD4
#define EA		PD5

#define RA		PC5
#define LA		PC4
#define RB		PC3
#define LB		PC2

#define BUZZ	PC6

#define SES		PB0
#define SST		PB1
#define ES		PB2
#define ST		PB3

#define BAT		PA4

#define Filt_order			6

#define ENCODER_TYPE		990.0
#define WHEELS_DIA			6.4
#define PI					3.14159
#define DECELERATE_PT		300
#define DECELERATE_PTLR		300
#define SPEED_CONST			1485										//
#define START_SPEED			32											// old 32
#define PWM_MAX				799
#define RPM_MAX				128.0
#define RPM_MIN				6.0
#define ACC_CONST			2.48										//Accelerated constant		//2.48
#define DEC_CONST			2.48										//Accelerated constant
#define MAX_RANGE			5000.0										//Max running range

#define STEER_COEF			1.1
#define STEER_INC			2.08
#define WHEELS_DISTANCE		22.5										//Distance between two wheels

#define TURNL_COEF			1.0
#define TURNR_COEF			1.015
#define TURNB_COEF			1.11

// #define TURNL_COEF			0.988
// #define TURNR_COEF			0.995
// #define TURNB_COEF			1.04

/*Robot directions  */
#define TURN				't'
#define FORWARD				'f'
#define BACKWARD			'b'
#define TURN_RIGHT			'r'
#define TURN_LEFT			'l'
#define TURN_BACK			'i'
#define STAY				's'

/*Robot behaviors  */
#define ACCELERATE			'a'
#define UNI_MOTION			'u'
#define DECELERATE			'd'
#define STOPPED				's'
#define STEER				's'

/*Robot done bit  */
#define DONE_CMD			'd'
#define READY_FOR_NEW_CMD	'r'
#define NOT_YET				'n'
#define ERROR				'e'

/*PID parameters  */
#define dt					0.0005

#define Kp_a				1.5										
#define Ki_a				0.75
#define Kd_a				0.004

#define Kp_u				1.2								
#define Ki_u				2.5
#define Kd_u				0.012

#define Kp_d				0.1
#define Ki_d				1.7
#define Kd_d				0.02






/*Response case  */
#define EVENT_BUFFER_SIZE	8
#define ES_UPDATE_INTERVAL	4000
#define RB_INITIALIZE		0
#define RB_EMERGENCY_STOP	1
#define RB_TEMPORARILY_STOP	2
#define RB_SET_START		3
#define RB_CONNECTED		4
#define RB_DISCONNECTED		5
#define RB_PID_A			6
#define RB_PID_B			7
#define RB_FORWARD			8
#define RB_BACKWARD			9
#define RB_STEERRIGHT		10
#define RB_STEERLEFT		11
#define RB_TURNRIGHT		12
#define RB_TURNLEFT			13
#define RB_TURNBACK			14
#define RB_FWD_ACCELERATE	15
#define RB_FWD_DECELERATE	16

#define RB_SPEED			98
#define RB_DONE				99

#define CMD_01				101											//Speed Return Enable
#define CMD_02				102											//Speed Return Disable
#define CMD_03				103											//Encoder Return Enable
#define CMD_04				104											//Encoder Return Disable
#define CMD_05				105											//Buzzer enabled
#define CMD_06				106											//Buzzer disabled
#define ERR_01				201											//Battery is running low
#define ERR_02				202											//Cannot decelerate while stopping
#define ERR_03				203											//Invalid forward command




/*ADC parameters  */
#define ADC_INTERVAL		250
#define UPPER_BOUNDED		756											//790 -50
#define LOWER_BOUNDED		665											//690 -50
#define ADC_BIAS			658											//680-50
#define A2_poly				0.0069
#define A1_poly				0.3551
#define A0_poly				-1.72


#define MOTOR_A				'a'
#define MOTOR_B				'b'



#ifndef MYROBOT_H_
#define MYROBOT_H_

typedef struct{
	int8_t DIR;
	int32_t val;
	uint8_t state;
	uint16_t cap, ovf_times;
	float pps;
	float (*GetSpeed)(void);
	int32_t (*GetValue)(void);
}Encoder;

typedef struct{
	uint8_t state,frame;
	float er, last_er, sum_er, dif_er;
	int32_t er_control;
	float REF;
	void (*loop)(void);
}PID_parameters;

typedef struct{
	float sum, vec;
	float val[Filt_order];
}Mean_filter;

typedef struct{
	Encoder EN;
	PID_parameters PID;
	Mean_filter MFilt;
	void (*Mean_filt)(void);
}Motor;

Motor MB,MA;

typedef struct{
	void (*cm)(float);
	void (*m)(float);
	void (*rounds)(float);
	void (*Accelerate)(void);
	void (*Decelerate)(void);
}FWD;

typedef struct{
	void (*cm)(float);
	void (*m)(float);
	void (*rounds)(float);
}BWD;

typedef struct{
	void (*Right)(void);
	void (*Left)(void);
}Steering;

typedef struct{
	void (*Right)(void);
	void (*Left)(void);
	void (*Back)(void);
}Turning;

/*
 * Direction:  
		+ f: Forward
		+ b: Backward
		+ r: Turn right
		+ l: Turn left
		+ i: Turn back
		+ s: Stay
 * Behavior: (speed)
		+ a: Accelerate 
		+ d: Decelerate 
		+ u: Uni-motion
		+ s: Stay
 * Done_bit:
		+ n: Not yet
		+ d: Done
		+ r: Ready
		+ e: Error
 */
typedef struct{
	char Direction, Behavior, L_Behavior, Done_bit; 
	int32_t dec_point;
	int32_t duty;
	void (*Accelerate)(void);
	void (*UniMotion)(void);
	void (*Decelerate)(void);
	void (*Check_stop)(void);
	void (*PID_loop)(void);
	uint8_t (*GetBatPercent)(void);
	void (*Stop)(void);
	void (*EStop)(void);
	void (*SetSTART)(uint8_t __start);
	uint8_t (*GetESTOP)(void);
	uint8_t (*GetST_Button)(void);
	uint8_t (*GetConnection)(void);
	void (*Response)(uint8_t _case);
	void (*EventHandler)(void);
	FWD Forward;
	BWD Backward;
	Steering Steer;
	Turning Turn;
}Robot_struct;

Robot_struct Robot;

void IO_Init(void);
void ADC_Init(void);
void ADC_Trig(uint8_t Adc_channel);
uint8_t ADCtoPercent(uint16_t _ADC_val);
uint8_t Robot_GetBatPercent(void);
void Fast_PWM_20KHz_Init(void);
void Timer0_CTC_Init(void);
void Timer2_CTC_Init(void);
void Ex_INT_Init(void);
void MATrig(int8_t __DIR);
void MBTrig(int8_t __DIR);
float MA_GetSpeed(void);
float MB_GetSpeed(void);
int32_t MA_GetValue(void);
int32_t MB_GetValue(void);
void PID_TrigParameters(char _state);
void PID_Clear(char _motor);
void PID_A_SetValue(float __rpm);
void PID_B_SetValue(float __rpm);
void PID_SetValues(float __rpmA, float __rpmB);
void PID_A_Work(void);
void PID_B_Work(void);
void PID_Loop_B(void);
void Robot_PID_Loops(void);
void Robot_Check_stop(void);
void Mean_filt_MA(void);
void Mean_filt_MB(void);
void Stop(void);
void E_Stop(void);
uint8_t Check_Button(uint8_t BS, uint8_t L_BS);
void Led_Ind(uint8_t _state);
uint8_t Robot_GetST_Button(void);
void Robot_SetSTART(uint8_t __start);
uint8_t Robot_GetESTOP(void);
void DecelerateDone(void);
void Robot_Accelerate(void);
void Robot_UniMotion(void);
void Robot_Decelerate(void);
void Robot_Forward_cm(float _cm);
void Robot_Forward_m(float __m);
void Robot_Forward_r(float __r);
void Robot_Backward_cm(float _cm);
void Robot_Backward_m(float __m);
void Robot_Backward_r(float __r);
void Robot_RightSteer(void);
void Robot_LeftSteer(void);
void Robot_SetUpTurn(uint8_t _dir);
void Robot_Turn_Right(void);
void Robot_Turn_Left(void);
void Robot_Turn_Back(void);
void Robot_Forward_Accelerate(void);
void Robot_Forward_Decelerate(void);
void Robot_Init(void);
uint8_t Robot_GetConnection(void);
void Robot_EventBufferClear(void);
void Robot_Response(uint8_t _case);
void Robot_EventHandler(void);

uint32_t millis(void);





#endif /* MYROBOT_H_ */