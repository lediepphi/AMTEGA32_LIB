
#include "MyRobot.h"

uint8_t i_meanA = 0, i_meanB = 0, i_ADC = 0, j, START, ESTOP, ST_But, L_ST_But, R_Steer, L_Steer, severConnection = 0, speedReturn = 0, encoderReturn = 0, buzzerEnable = 1, percentBat = 50, lastPercentBat;
uint16_t __captureA, __captureB, _currentADCValue;
uint32_t _millis_AMR = 0, _lastMillis = 0, _ADCMillis = 0;
float Kp,Kd,Ki;
uint8_t robotAutoAUD = 0, i_buzz = 0;

char buff[EVENT_BUFFER_SIZE];
char buff_val[EVENT_BUFFER_SIZE];
char pps_vals[18];

void IO_Init(void){
	DDRB |= (1<<SST) | (1<<SES);
	PORTB &= ~((1<<SST) | (1<<SES));
	DDRD |= (1<<EA) | (1<<EB);
	PORTD &= ~((1<<EA) | (1<<EB));
	DDRC |= (1<<LA) | (1<<RA) | (1<<LB) | (1<<RB) | (1<<BUZZ);
	PORTC &= ~((1<<LA) | (1<<RA) | (1<<LB) | (1<<RB) | (1<<BUZZ));
}

void ADC_Init(void){
	ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADMUX |= (0<<REFS1) | (1<<REFS0);
}

void ADC_Trig(uint8_t Adc_channel){
	if(Adc_channel > 8 ) return;
	ADMUX |= Adc_channel;
	ADCSRA |= (1<<ADSC);
}

uint8_t ADCtoPercent(uint16_t _ADC_val){
	if(_ADC_val > UPPER_BOUNDED) return 100;
	if(_ADC_val < LOWER_BOUNDED) return 1;
	double _percent;
	_ADC_val = _ADC_val - ADC_BIAS;
	_percent = A2_poly*_ADC_val*_ADC_val + A1_poly*_ADC_val + A0_poly;
	return ((int8_t) _percent);
}

uint8_t Robot_GetBatPercent(void){
	return lastPercentBat;
}

void Fast_PWM_20KHz_Init(void){
	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);	
	TCNT1 = 0;	
	OCR1A = PWM_MAX;
	OCR1B = PWM_MAX;
	ICR1 = PWM_MAX;
	TIMSK |= (1<<TOIE1);
}

void Timer0_CTC_Init(void){ 
	TCCR0 |= (1<<WGM01) | (1<<CS01) | (1<<CS00);
	TIMSK |= (1<<OCIE0); 
	OCR0 = 250;
	TCNT0 = 0;
}

void Timer2_CTC_Init(void){ 
	TCCR2 |= (1<<WGM21) | (1<<CS22);
	TIMSK |= (1<<OCIE2); 
	OCR2 = 250;
	TCNT2 = 0;
}

void Ex_INT_Init(void){
	GICR  |= (1<<INT2) | (1<<INT1) | (1<<INT0);
	MCUCR |= (1<<ISC01)| (1<<ISC11);
	MCUCSR &= ~(1<<ISC2);
}

void buzzerTrig(uint8_t _state){
	_state = _state&buzzerEnable;
	if(_state  == 1){
		PORTC |= (1<<BUZZ);
	}
	else if(_state  == 0){
		PORTC &= ~(1<<BUZZ);
	}
}

void buzzerControl(void){
	if(percentBat < 10){
		i_buzz++;
		if(i_buzz < 3) buzzerTrig(1);
		else if(i_buzz < 4) buzzerTrig(0);
		else i_buzz = 0;
	}
	else buzzerTrig(0);
}

void MATrig(int8_t __DIR){
	MA.EN.DIR = __DIR;
	if(__DIR == 1){
		PORTC &= ~(1<<LA);
		PORTC |= (1<<RA);
	}
	else if(__DIR == -1){
		PORTC &= ~(1<<RA);
		PORTC |= (1<<LA);
	}
	else{
		PORTC &= ~((1<<LA) | (1<<RA));
		OCR1A = PWM_MAX;
		MA.EN.pps = 0;
		Mean_filt_MA();
		PORTD |= (1<<EA);
	}
}

void MBTrig(int8_t __DIR){
	MB.EN.DIR = __DIR;
	if(__DIR == 1){
		PORTC &= ~(1<<RB);
		PORTC |= (1<<LB);
	}
	else if(__DIR == -1){
		PORTC &= ~(1<<LB);
		PORTC |= (1<<RB);
	}
	else{
		PORTC &= ~((1<<LB) | (1<<RB));
		OCR1B = PWM_MAX;
		MB.EN.pps = 0;
		Mean_filt_MB();
		PORTD |= (1<<EB);
	}
}

float MA_GetSpeed(void){
	return MA.MFilt.vec;
}

float MB_GetSpeed(void){
	return MB.MFilt.vec;
}

int32_t MA_GetValue(void){
	return MA.EN.val;
}

int32_t MB_GetValue(void){
	return MB.EN.val;
}

void PID_TrigParameters(char _state){
	if(_state == ACCELERATE){
		Kp = Kp_a;
		Ki = Ki_a;
		Kd = Kd_a;
	}
	else if(_state == UNI_MOTION){
		Kp = Kp_u;
		Ki = Ki_u;
		Kd = Kd_u;
	}
	else if(_state == DECELERATE){
		Kp = Kp_d;
		Ki = Ki_d;
		Kd = Kd_d;
	}
	else if(_state == STOPPED){
		Kp = 0;
		Ki = 0;
		Kd = 0;
	}

}

void PID_Clear(char _motor){
	if(_motor == MOTOR_A){
		MA.PID.last_er = 0;
		MA.PID.sum_er = 0;
		MA.PID.dif_er = 0;
		MA.PID.frame = 0;
		MA.EN.ovf_times = 0;
	}
	else if(_motor == MOTOR_B){
		MB.PID.last_er = 0;
		MB.PID.sum_er = 0;
		MB.PID.dif_er = 0;
		MB.PID.frame = 3;
		MB.EN.ovf_times = 0;
	}
}

void PID_A_SetValue(float __rpm){
	if(__rpm == 0){
		MATrig(0);
		MA.PID.state = 0;
		return;
	}
	float check_rpm = fabs(__rpm);
	if((check_rpm > RPM_MAX) || (check_rpm < RPM_MIN)) return;
	MA.PID.REF = (__rpm/60)*ENCODER_TYPE;
	MA.PID.er = MA.PID.REF;
	PID_Clear(MOTOR_A);
	MA.PID.state = 1;
}

void PID_B_SetValue(float __rpm){
	if(__rpm == 0){
		MBTrig(0);
		MB.PID.state = 0;
		return;
	}
	float check_rpm = fabs(__rpm);
	if((check_rpm > RPM_MAX) || (check_rpm < RPM_MIN)) return;
	MB.PID.REF = (__rpm/60)*ENCODER_TYPE;
	MB.PID.er = MB.PID.REF;
	PID_Clear(MOTOR_B);
	MB.PID.state = 1;
}

void PID_SetValues(float __rpmA, float __rpmB){
// 	PID_A_SetValue(__rpmA);
// 	PID_B_SetValue(__rpmB);
	float check_rpm;
	if(__rpmA == 0){
		MATrig(0);
		MA.PID.state = 0;
		return;
	}
	if(__rpmB == 0){
		MBTrig(0);
		MB.PID.state = 0;
		return;
	}
	check_rpm = fabs(__rpmA);
	if((check_rpm > RPM_MAX) || (check_rpm < RPM_MIN)) return;
	check_rpm = fabs(__rpmB);
	if((check_rpm > RPM_MAX) || (check_rpm < RPM_MIN)) return;
	
	MA.PID.REF = (__rpmA/60)*ENCODER_TYPE;
	MB.PID.REF = (__rpmB/60)*ENCODER_TYPE;
	MA.PID.er = MA.PID.REF;
	MB.PID.er = MB.PID.REF;
	PID_Clear(MOTOR_A);
	PID_Clear(MOTOR_B);
	MB.PID.state = 1;
	MA.PID.state = 1;
}	

void PID_A_Work(void){
	MA.PID.sum_er = MA.PID.sum_er + MA.PID.er*dt;
	MA.PID.dif_er = (MA.PID.er - MA.PID.last_er)/dt;
	MA.PID.er_control = MA.PID.er*Kp + MA.PID.sum_er*Ki + MA.PID.dif_er*Kd;
	if(MA.PID.er_control > 0){
		MATrig(1);
		if(MA.PID.er_control > PWM_MAX) OCR1A = PWM_MAX;
		else OCR1A = MA.PID.er_control;
	}
	else{
		MATrig(-1);
		if(MA.PID.er_control < -PWM_MAX) OCR1A = PWM_MAX;
		else OCR1A = -MA.PID.er_control;
	}
	MA.PID.last_er = MA.PID.er;
}

void PID_B_Work(void){
	MB.PID.sum_er = MB.PID.sum_er + MB.PID.er*dt;
	MB.PID.dif_er = (MB.PID.er - MB.PID.last_er)/dt;
	MB.PID.er_control = MB.PID.er*Kp + MB.PID.sum_er*Ki + MB.PID.dif_er*Kd;
	if(MB.PID.er_control > 0){
		MBTrig(1);
		if(MB.PID.er_control > PWM_MAX) OCR1B = PWM_MAX;
		else OCR1B = MB.PID.er_control;
	}
	else{
		MBTrig(-1);
		if(MB.PID.er_control < -PWM_MAX) OCR1B = PWM_MAX;
		else OCR1B = -MB.PID.er_control;
	}
	MB.PID.last_er = MB.PID.er;
}

void PID_Loop_A(void){
	MA.PID.frame++;
	if(MA.PID.frame >= 5){
		MA.PID.er = MA.PID.REF - MA.MFilt.vec;
		if(MA.PID.state) PID_A_Work();
		MA.PID.frame = 0;
	}
}

void PID_Loop_B(void){
	MB.PID.frame++;
	if(MB.PID.frame >= 5){
		MB.PID.er = MB.PID.REF - MB.MFilt.vec;
		if(MB.PID.state) PID_B_Work();
		MB.PID.frame = 0;
	}
}

void Robot_PID_Loops(void){
	if(MA.PID.state) PID_Loop_A();
	if(MB.PID.state) PID_Loop_B();
}

void Robot_Check_stop(void){
	MB.EN.ovf_times++;
	MA.EN.ovf_times++;
	if(MA.EN.ovf_times >= 200){
		MA.EN.pps = 0;
		MA.EN.ovf_times = 0;
		MA.EN.DIR = 0;
		MA.Mean_filt();
	}
	if(MB.EN.ovf_times >= 200){
		MB.EN.pps = 0;
		MB.EN.ovf_times = 0;
		MB.EN.DIR = 0;
		MB.Mean_filt();
	}
}

void Mean_filt_MA(void){
	MA.MFilt.sum = MA.MFilt.sum - MA.MFilt.val[i_meanA];
	MA.MFilt.sum = MA.MFilt.sum + MA.EN.pps;
	MA.MFilt.val[i_meanA] = MA.EN.pps;
	i_meanA = (i_meanA + 1)%Filt_order;
	MA.MFilt.vec = MA.MFilt.sum/Filt_order;
}

void Mean_filt_MB(void){
	MB.MFilt.sum = MB.MFilt.sum - MB.MFilt.val[i_meanB];
	MB.MFilt.sum = MB.MFilt.sum + MB.EN.pps;
	MB.MFilt.val[i_meanB] = MB.EN.pps;
	i_meanB = (i_meanB + 1)%Filt_order;
	MB.MFilt.vec = MB.MFilt.sum/Filt_order;
}

void Stop(void){
	PORTC &= ~((1<<LA) | (1<<RA) | (1<<LB) | (1<<RB));
	MA.PID.state = 0;
	MB.PID.state = 0;
	MBTrig(0);
	MATrig(0);
	_delay_ms(20);
	Robot.L_Behavior = Robot.Behavior;
	Robot.Behavior = STOPPED;
	Robot.Direction = STAY;
	Robot.Done_bit = DONE_CMD;
}	

void E_Stop(void){
	ESTOP = 1;
	START = 0;
	Led_Ind(START);
	DecelerateDone();
}

uint8_t Check_Button(uint8_t BS, uint8_t L_BS){
	if((L_BS - BS) == 1) return 1;
	else return 0;
}

void Led_Ind(uint8_t _state){
	if(_state){
		PORTB |= (1<<SST);
		PORTB &= ~(1<<SES);
	}
	else{
		PORTB |= (1<<SES);
		PORTB &= ~(1<<SST);
	}
}

uint8_t Robot_GetST_Button(void){
	return ST_But;
}

void Robot_SetSTART(uint8_t __start){
	if(lastPercentBat < 10){
		Robot.Response(ERR_01);
		Robot.Done_bit = ERROR;
		return;
	}
	START = __start&1;
	ESTOP = (~__start)&1;
	Robot.Done_bit = DONE_CMD;
	UART.Clear_Buffer();
}

uint8_t Robot_GetESTOP(void){
	return ESTOP;
}

void DecelerateDone(void){
	Stop();
	Stop();
	//Robot.Response(RB_SPEED);
	PID_TrigParameters(STOPPED);
	_delay_ms(50);
	Robot.Done_bit = DONE_CMD;
	robotAutoAUD = 0;
}

void Robot_Accelerate(void){
	if(Robot.Direction == FORWARD){
		if((MA.MFilt.vec < SPEED_CONST) && (MB.MFilt.vec < SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF + ACC_CONST;
			MB.PID.REF = MB.PID.REF + ACC_CONST;
		}
		else{
			MA.PID.REF = SPEED_CONST;
			MB.PID.REF = SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = UNI_MOTION;
			PID_TrigParameters(UNI_MOTION);
			if(!robotAutoAUD) Robot.Done_bit = DONE_CMD;
		}
	}
	else if(Robot.Direction == BACKWARD){
		if((MA.MFilt.vec > -SPEED_CONST) && (MB.MFilt.vec > -SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF - ACC_CONST;
			MB.PID.REF = MB.PID.REF - ACC_CONST;
		}
		else{
			MA.PID.REF = -SPEED_CONST;
			MB.PID.REF = -SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = UNI_MOTION;
			PID_TrigParameters(UNI_MOTION);
		}
	}
	else if((Robot.Direction == TURN_RIGHT) || (Robot.Direction == TURN_BACK)){
		if((MA.MFilt.vec > -SPEED_CONST) || (MB.MFilt.vec < SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF - ACC_CONST;
			MB.PID.REF = MB.PID.REF + ACC_CONST;
		}
		else{
			MA.PID.REF = -SPEED_CONST;
			MB.PID.REF = SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = UNI_MOTION;
			PID_TrigParameters(UNI_MOTION);
		}
	}
	else if(Robot.Direction == TURN_LEFT){
		if((MA.MFilt.vec < SPEED_CONST) || (MB.MFilt.vec > -SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF + ACC_CONST;
			MB.PID.REF = MB.PID.REF - ACC_CONST;
		}
		else{
			MA.PID.REF = SPEED_CONST;
			MB.PID.REF = -SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = UNI_MOTION;
			PID_TrigParameters(UNI_MOTION);
		}
	}
}

void Robot_UniMotion(void){
	if(Robot.Direction == FORWARD){
		if(R_Steer){
			if(MA.PID.REF < MB.PID.REF){
				MA.PID.REF = MA.PID.REF + STEER_INC;
				MB.PID.REF = MB.PID.REF - STEER_INC;
			}
			else{
				MA.PID.REF = SPEED_CONST;
				MB.PID.REF = MA.PID.REF;
				R_Steer = 0;
			}
		}
		else if(L_Steer){
			if(MB.PID.REF < MA.PID.REF){
				MB.PID.REF = MB.PID.REF + STEER_INC;
				MA.PID.REF = MA.PID.REF - STEER_INC;
			}
			else{
				MB.PID.REF = SPEED_CONST;
				MA.PID.REF = MB.PID.REF;
				L_Steer = 0;
			}
		}
		if(!robotAutoAUD) return;
		if((MA.EN.val > Robot.dec_point) || (MB.EN.val > Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = DECELERATE;
			R_Steer = 0;
			L_Steer = 0;
			PID_TrigParameters(DECELERATE);
		}
	}
	else if(Robot.Direction == BACKWARD){
		if((MA.EN.val < Robot.dec_point) || (MB.EN.val < Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = DECELERATE;
			R_Steer = 0;
			L_Steer = 0;
			PID_TrigParameters(DECELERATE);
		}
	}
	else if((Robot.Direction == TURN_RIGHT) || (Robot.Direction == TURN_BACK)){
		if((MA.EN.val < -Robot.dec_point) || (MB.EN.val > Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = DECELERATE;
			PID_TrigParameters(DECELERATE);
		}
	}
	else if(Robot.Direction == TURN_LEFT){
		if((MA.EN.val > Robot.dec_point) || (MB.EN.val < - Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = DECELERATE;
			PID_TrigParameters(DECELERATE);
		}
	}
}

void Robot_Decelerate(void){
	if(robotAutoAUD  == 0){
		if(Robot.Direction == FORWARD){
			if((MA.PID.REF < 400) && (MB.PID.REF < 400)) {
				DecelerateDone();
			}
			else{
				MA.PID.REF = MA.PID.REF - DEC_CONST;
				MB.PID.REF = MB.PID.REF - DEC_CONST;
			}
		}
		return;
	}
	
	if(Robot.Direction == FORWARD){
		if((MA.EN.val >= Robot.duty) || (MB.EN.val >= Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF - DEC_CONST;
			MB.PID.REF = MB.PID.REF - DEC_CONST;
		}
	}
	else if(Robot.Direction == BACKWARD){
		if((MA.EN.val <= Robot.duty) || (MB.EN.val <= Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF + DEC_CONST;
			MB.PID.REF = MB.PID.REF + DEC_CONST;
		}
	}
	else if((Robot.Direction == TURN_RIGHT) || (Robot.Direction == TURN_BACK)){
		if((MA.EN.val <= -Robot.duty) || (MB.EN.val >= Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF + DEC_CONST;
			MB.PID.REF = MB.PID.REF - DEC_CONST;
		}
	}
	else if(Robot.Direction == TURN_LEFT){
		if((MA.EN.val >= Robot.duty) || (MB.EN.val <= -Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF - DEC_CONST;
			MB.PID.REF = MB.PID.REF + DEC_CONST;
		}
	}	
}

void Robot_Forward_cm(float _cm){
	if(Robot.Behavior != STOPPED) return;
	if((_cm > MAX_RANGE) || (_cm < 0)) return;
	Robot.duty = (_cm/(PI*WHEELS_DIA))*ENCODER_TYPE;
	Robot.dec_point = Robot.duty - DECELERATE_PT;
	Robot.Direction = FORWARD;
	Robot.Behavior = ACCELERATE;
	Robot.L_Behavior = STOPPED;
	Robot.Done_bit = NOT_YET;
	MA.EN.val = 0;
	MB.EN.val = 0;
	robotAutoAUD = 1;
	PID_TrigParameters(ACCELERATE);
	PID_SetValues(START_SPEED,START_SPEED);
	
}

void Robot_Forward_m(float __m){
	__m = __m*100;
	Robot_Forward_cm(__m);
}

void Robot_Forward_r(float __r){
	__r = __r*PI*WHEELS_DIA;
	Robot_Forward_cm(__r);
}

void Robot_Backward_cm(float _cm){
	if(Robot.Behavior != STOPPED) return;
	if((_cm > MAX_RANGE) || (_cm < 0)) return;
	Robot.duty = -(_cm/(PI*WHEELS_DIA))*ENCODER_TYPE;
	Robot.dec_point = Robot.duty + DECELERATE_PT;
	Robot.Direction = BACKWARD;
	Robot.Behavior = ACCELERATE;
	Robot.L_Behavior = STOPPED;
	Robot.Done_bit = NOT_YET;
	MA.EN.val = 0;
	MB.EN.val = 0;
	robotAutoAUD = 1;
	PID_TrigParameters(ACCELERATE);
	PID_SetValues(-START_SPEED,-START_SPEED);
}

void Robot_Backward_m(float __m){
	__m = __m*100;
	Robot_Backward_cm(__m);
}

void Robot_Backward_r(float __r){
	__r = __r*PI*WHEELS_DIA;
	Robot_Backward_cm(__r);
}

void Robot_RightSteer(void){
	if(Robot.Behavior == UNI_MOTION) {
		R_Steer = 1;
		if(Robot.Direction == FORWARD){
			MA.PID.REF = MA.PID.REF/STEER_COEF;
			MB.PID.REF = MB.PID.REF*STEER_COEF;
		}
	}
}

void Robot_LeftSteer(void){
	if(Robot.Behavior == UNI_MOTION) {
		L_Steer = 1;
		if(Robot.Direction == FORWARD){
			MB.PID.REF = MB.PID.REF/STEER_COEF;
			MA.PID.REF = MA.PID.REF*STEER_COEF;
		}
	}
}

void Robot_SetUpTurn(uint8_t _dir){
	Robot.Behavior = ACCELERATE;
	Robot.L_Behavior = STOPPED;
	Robot.Done_bit = NOT_YET;
	MA.EN.val = 0;
	MB.EN.val = 0;
	robotAutoAUD = 1;
	if(_dir == TURN_RIGHT){
		Robot.Direction = TURN_RIGHT;
		Robot.duty = (WHEELS_DISTANCE*TURNR_COEF/(4*WHEELS_DIA))*ENCODER_TYPE;
		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
		PID_TrigParameters(ACCELERATE);
		PID_SetValues(-START_SPEED,START_SPEED);
		
	}
	else if(_dir == TURN_LEFT){
		Robot.Direction = TURN_LEFT;
		Robot.duty = (WHEELS_DISTANCE*TURNL_COEF/(4*WHEELS_DIA))*ENCODER_TYPE;
		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
		PID_TrigParameters(ACCELERATE);
		PID_SetValues(START_SPEED,-START_SPEED);
	}
	else if(_dir == TURN_BACK){
		Robot.Direction = TURN_BACK;
		Robot.duty = (WHEELS_DISTANCE*TURNR_COEF/(2*WHEELS_DIA))*ENCODER_TYPE;
		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
		PID_TrigParameters(ACCELERATE);
		PID_SetValues(-START_SPEED,START_SPEED);
	}		
	
}

void Robot_Turn_Right(void){
	if(Robot.Behavior != STOPPED) return;
	Robot_SetUpTurn(TURN_RIGHT);
}

void Robot_Turn_Left(void){
	if(Robot.Behavior != STOPPED) return;
	Robot_SetUpTurn(TURN_LEFT);
}

void Robot_Turn_Back(void){
	if(Robot.Behavior != STOPPED) return;
	Robot_SetUpTurn(TURN_BACK);
}

void Robot_Forward_Accelerate(void){
	if(Robot.Behavior != STOPPED) return;
	Robot.Direction = FORWARD;
	Robot.Behavior = ACCELERATE;
	Robot.L_Behavior = STOPPED;
	Robot.Done_bit = NOT_YET;
	MA.EN.val = 0;
	MB.EN.val = 0;
	robotAutoAUD = 0;
	PID_TrigParameters(ACCELERATE);
	PID_SetValues(START_SPEED,START_SPEED);
}

void Robot_Forward_Decelerate(void){
	if(Robot.Direction == FORWARD){
		robotAutoAUD = 0;
		Robot.L_Behavior = UNI_MOTION;
		Robot.Behavior = DECELERATE;
		PID_TrigParameters(DECELERATE);
		Robot.Done_bit = NOT_YET;
	}
}

void Robot_Init(void){
	MA.Mean_filt = &Mean_filt_MA;
	MB.Mean_filt = &Mean_filt_MB;
	MA.PID.loop = &PID_Loop_A;
	MB.PID.loop = &PID_Loop_B;
	MA.EN.GetSpeed = &MA_GetSpeed;
	MB.EN.GetSpeed = &MB_GetSpeed;
	MA.EN.GetValue = &MA_GetValue;
	MB.EN.GetValue = &MB_GetValue;
	Robot.GetBatPercent = &Robot_GetBatPercent;
	Robot.GetST_Button = &Robot_GetST_Button;
	Robot.SetSTART = &Robot_SetSTART;
	Robot.GetESTOP = &Robot_GetESTOP;
	Robot.Forward.cm = &Robot_Forward_cm;
	Robot.Forward.m = &Robot_Forward_m;
	Robot.Forward.rounds = &Robot_Forward_r;
	Robot.Forward.Accelerate = &Robot_Forward_Accelerate;
	Robot.Forward.Decelerate = &Robot_Forward_Decelerate;
	Robot.Backward.cm = &Robot_Backward_cm;
	Robot.Backward.m = &Robot_Backward_m;
	Robot.Backward.rounds = &Robot_Backward_r;
	Robot.Accelerate = &Robot_Accelerate;
	Robot.UniMotion = &Robot_UniMotion;
	Robot.Decelerate = &Robot_Decelerate;	
	Robot.Check_stop = &Robot_Check_stop;
	Robot.PID_loop = &Robot_PID_Loops;
	Robot.Stop = &Stop;
	Robot.EStop = &E_Stop;
	Robot.Steer.Right = &Robot_RightSteer;
	Robot.Steer.Left = &Robot_LeftSteer;
	Robot.Turn.Right = &Robot_Turn_Right;
	Robot.Turn.Left = &Robot_Turn_Left;
	Robot.Turn.Back = &Robot_Turn_Back;
	Robot.GetConnection = &Robot_GetConnection;
	Robot.Response = &Robot_Response;
	Robot.EventHandler = &Robot_EventHandler;
	Robot.Direction = STAY;
	Robot.Behavior = STOPPED;
	
	MA.MFilt.sum = 0;
	MB.MFilt.sum = 0;
	
	ADC_Init();
	Ex_INT_Init();
	Fast_PWM_20KHz_Init();
	Timer0_CTC_Init();
	Timer2_CTC_Init();
	UART_Init(38400);
	IO_Init();
	
	E_Stop();
	I2C_LCD_Init(I2C_FREQ,LS_NONE);
	sei();
	LCD.Clear();
	LCD.WriteString("Initialized...");
	Robot.Response(RB_INITIALIZE);	
	buzzerTrig(0);
	_delay_ms(400);
	lastPercentBat = percentBat;
}

uint8_t Robot_GetConnection(void){
	return severConnection;
}

void Robot_EventBufferClear(void){
	for(j = 0; j < EVENT_BUFFER_SIZE; j++){
		buff[j] = '\0';
		buff_val[j] = '\0';
	}
}

void setPIDnResponse(char _motor){
	int8_t speed;
	PID_TrigParameters(UNI_MOTION);
	buff_val[0] = buff[2];
	for(j = 0; j < 4; j++){
		if(!isdigit(buff[3 + j])) break;
		buff_val[j+1] = buff[3 + j];	
	}		
	buff_val[j+1] = '\0';		
	speed = atoi(buff_val);

	if(_motor == MOTOR_A){
		PID_A_SetValue(speed);
		Robot.Response(RB_PID_A);
	}
	else if(_motor == MOTOR_B){
		PID_B_SetValue(speed);
		Robot.Response(RB_PID_B);
	}
}

void Robot_Response(uint8_t _case){
	if(_case == RB_INITIALIZE){
//		UART.TX.String("Robot Initialized\n");
		UART.TX.String("\nAMR.I\n");
		return;
	}
	else if(_case == RB_EMERGENCY_STOP){
//		UART.TX.String("Robot Emergency Stopped\n");
		UART.TX.String("AMR.es\n");
		return;
	}
	else if(_case == RB_TEMPORARILY_STOP){
//		UART.TX.String("Robot Stopped\n");
		UART.TX.String("AMR.gg\n");	
		return;	
	}
	else if(_case == RB_SET_START){
//		UART.TX.String("Robot Started\n");
		UART.TX.String("AMR.go\n");	
		return;	
	}
	else if(_case == RB_CONNECTED){
//		UART.TX.String("Connected to server\n");
		UART.TX.String("AMR.cnt\n");
		return;
	}
	else if(_case == RB_DISCONNECTED){
//		UART.TX.String("Disconnected to server\n");
		UART.TX.String("AMR.dnt\n");	
		return;	
	}
	else if(_case == RB_PID_A){
// 		UART.TX.String("Set PID motor A: ");
// 		UART.TX.String(buff_val);
// 		UART.TX.NewLine();
		UART.TX.String("AMR.PA:");
		UART.TX.String(buff_val);
		UART.TX.NewLine();
		return;
	}
	else if(_case == RB_PID_B){
// 		UART.TX.String("Set PID motor B: ");
// 		UART.TX.String(buff_val);
// 		UART.TX.NewLine();
		UART.TX.String("AMR.PB:");
		UART.TX.String(buff_val);
		UART.TX.NewLine();
		return;
	}
	else if(_case == RB_FORWARD){
		UART.TX.String("AMR.f");
		UART.TX.String(buff_val);
		UART.TX.NewLine();
// 		UART.TX.String("Forward command: ");
// 		UART.TX.String(buff_val);
// 		UART.TX.NewLine();
		return;
	}
	else if(_case == RB_BACKWARD){
		UART.TX.String("AMR.b");
		UART.TX.String(buff_val);
		UART.TX.NewLine();
// 		UART.TX.String("Backward command: ");
// 		UART.TX.String(buff_val);
// 		UART.TX.NewLine();
		return;
	}
	else if(_case == RB_STEERRIGHT){
//		UART.TX.String("Steer Right\n");
		UART.TX.String("AMR.sr\n");
		return;
	}
	else if(_case == RB_STEERLEFT){
//		UART.TX.String("Steer Right\n");
		UART.TX.String("AMR.sl\n");
		return;
	}
	else if(_case == RB_TURNRIGHT){
//		UART.TX.String("Turn Right\n");
		UART.TX.String("AMR.tr\n");
		return;		
	}
	else if(_case == RB_TURNLEFT){
//		UART.TX.String("Turn Left\n");
		UART.TX.String("AMR.tl\n");	
		return;
	}
	else if(_case == RB_TURNBACK){
//		UART.TX.String("Turn Back\n");
		UART.TX.String("AMR.tb\n");	
		return;
	}
	else if(_case == RB_FWD_ACCELERATE){
//		UART.TX.String("Forward Accelerate\n");
		UART.TX.String("AMR.fa\n");
		return;		
	}
	else if(_case == RB_FWD_DECELERATE){
//		UART.TX.String("Forward Decelerate\n");
		UART.TX.String("AMR.fd\n");
		return;		
	}
	else if(_case == RB_SPEED){
		if(speedReturn){
			sprintf(pps_vals,"%6d,%6d;\n",((int16_t)MA.MFilt.vec), ((int16_t)MB.MFilt.vec));
//			sprintf(pps_vals,"%6d,%6d;\n",((int16_t)MA.EN.pps), ((int16_t)MB.EN.pps));
			UART.TX.String(pps_vals);	
		}
		return;
	}
	else if(_case == RB_DONE){
//		UART.TX.String("Done!!!\n");
		UART.TX.String("AMR.d\n");
		if(encoderReturn){
			char En_vals[26];
			UART.TX.String("Encoder stopped at: \n");
			sprintf(En_vals,"%11ld,%11ld;\n",MA.EN.val, MB.EN.val);
			UART.TX.String(En_vals);	
		}
		Robot.Done_bit = READY_FOR_NEW_CMD;
		return;
	}
	else if(_case == CMD_01){
//		UART.TX.String("Command 01 set\n");
		UART.TX.String("AMR.cmd01\n");	
		return;	
	}
	else if(_case == CMD_02){
//		UART.TX.String("Command 02 set\n");
		UART.TX.String("AMR.cmd02\n");
		return;		
	}
	else if(_case == CMD_03){
//		UART.TX.String("Command 03 set\n");
		UART.TX.String("AMR.cmd03\n");	
		return;	
	}
	else if(_case == CMD_04){
//		UART.TX.String("Command 04 set\n");
		UART.TX.String("AMR.cmd04\n");	
		return;	
	}
	else if(_case == CMD_05){
//		UART.TX.String("Command 05 set\n");
		UART.TX.String("AMR.cmd05\n");	
		return;	
	}
	else if(_case == CMD_06){
//		UART.TX.String("Command 06 set\n");
		UART.TX.String("AMR.cmd06\n");	
		return;	
	}
	else if(_case == ERR_01){
//		UART.TX.String("Battery is running low\n");
		UART.TX.String("AMR.er01\n");	
		return;	
	}
	else if(_case == ERR_02){
//		UART.TX.String("Cannot decelerate while stopping\n");
		UART.TX.String("AMR.er02\n");	
		return;	
	}
	else if(_case == ERR_03){
//		UART.TX.String("Invalid forward command\n");
		UART.TX.String("AMR.er03\n");	
		return;	
	}
}

void Robot_EventHandler(void){
	if(Robot.GetESTOP()){
		if((_millis_AMR - _lastMillis) > ES_UPDATE_INTERVAL){
			_lastMillis = _millis_AMR;
			Robot.Response(RB_EMERGENCY_STOP);
		}	
	}
	if(UART.RX.String_Done()){
		Robot_EventBufferClear();
		UART.RX.String(buff,EVENT_BUFFER_SIZE);
		if((buff[0] == 'e') && (buff[1] == 's')){
			Robot.Response(RB_EMERGENCY_STOP);
			Robot.EStop();
			while(!(Robot.Done_bit == DONE_CMD));
			return;
		}
		else if((buff[0] == 'c') && (buff[1] == 'n') && (buff[2] == 't')){
			severConnection = 1;
			Robot.Response(RB_CONNECTED);
			return;
		}
		else if((buff[0] == 'd') && (buff[1] == 'n') && (buff[2] == 't')){
			severConnection = 0;
			Robot.Response(RB_DISCONNECTED);
			return;
		}
		else if((buff[0] == 'g') && (buff[1] == 'g')){
			Robot.Response(RB_TEMPORARILY_STOP);
			Robot.Stop();
			_delay_ms(100);
			while(!(Robot.Done_bit == DONE_CMD));
			
			return;
		}
		else if((buff[0] == 'g') && (buff[1] == 'o')){
			Robot.Response(RB_SET_START);
			Robot.SetSTART(1);
			if(Robot.Done_bit == ERROR){
				Robot.Response(ERR_01);
				return;
			}
			while(!(Robot.Done_bit == DONE_CMD));
			return;
		}
		else if((buff[0] == 'c') && (buff[1] == 'm') && (buff[2] == 'd') && (buff[3] == '0')){
			if(buff[4] == '1'){
				speedReturn = 1;
				Robot.Response(CMD_01);
			}
			else if(buff[4] == '2'){
				speedReturn = 0;
				Robot.Response(CMD_02);
			}
			else if(buff[4] == '3'){
				encoderReturn = 1;
				Robot.Response(CMD_03);
			}
			else if(buff[4] == '4'){
				encoderReturn = 0;
				Robot.Response(CMD_04);
			}
			else if(buff[4] == '5'){
				buzzerEnable = 1;
				Robot.Response(CMD_05);
			}
			else if(buff[4] == '6'){
				buzzerEnable = 0;
				Robot.Response(CMD_06);
			}
			return;
		}
		if(Robot.GetESTOP()) return;
		if((buff[0] == 'P') && (buff[1] == 'A')){
			setPIDnResponse(MOTOR_A);
			return;
		}
		else if((buff[0] == 'P') && (buff[1] == 'B')){
			setPIDnResponse(MOTOR_B);
			return;
		}
		if(Robot.Direction == STOPPED){
			if((buff[0] == FORWARD) && (buff[1] == ACCELERATE)){
				Robot.Forward.Accelerate();
				Robot.Response(RB_FWD_ACCELERATE);
				return;
			}
			if((buff[0] == FORWARD) && (!isdigit(buff[1]))){
				if(buff[1] == DECELERATE) Robot.Response(ERR_02);
				else Robot.Response(ERR_03);
				return;
			}
		}
		else if((Robot.Behavior == UNI_MOTION) || (Robot.Behavior == ACCELERATE)){
			if((buff[0] == FORWARD) && (buff[1] == DECELERATE)){
				Robot.Forward.Decelerate();
				Robot.Response(RB_FWD_DECELERATE);
				return;
			}
		}
		if(Robot.Behavior == UNI_MOTION){
			if(buff[0] == STEER){
				if(buff[1] == TURN_RIGHT) {
					Robot.Steer.Right();
					Robot.Response(RB_STEERRIGHT);
				}
				if(buff[1] == TURN_LEFT){
					Robot.Steer.Left();
					Robot.Response(RB_STEERLEFT);
				}
			}
			return;
		}
		
		if(Robot.Done_bit == READY_FOR_NEW_CMD){
			uint16_t distance = 0;
			if(buff[0] == FORWARD){
				for(j = 1; j < EVENT_BUFFER_SIZE; j++){
					if(!isdigit(buff[j])) break;
					buff_val[j-1] = buff[j];	
				}		
				buff_val[j-1] = '\0';		
				distance = atoi(buff_val);
				if(distance < 10){
					Robot.Response(ERR_03);
					return;
				}
 				Robot.Forward.cm(distance);
				Robot.Response(RB_FORWARD);
			}
			else if(buff[0] == BACKWARD){
				for(j = 1; j < EVENT_BUFFER_SIZE; j++){
					if(!isdigit(buff[j])) break;
					buff_val[j-1] = buff[j];	
				}				
				buff_val[j-1] = '\0';
				distance = atoi(buff_val);
				if(distance < 10){
					Robot.Response(ERR_03);
					return;
				}
				Robot.Backward.cm(distance);
				Robot.Response(RB_BACKWARD);
			}
			else if(buff[0] == TURN){
				if(buff[1] == TURN_RIGHT){
					Robot.Turn.Right();
					Robot.Response(RB_TURNRIGHT);
				}
				if(buff[1] == TURN_LEFT){
					Robot.Turn.Left();
					Robot.Response(RB_TURNLEFT);
				}
				if(buff[1] == 'b'){
					Robot.Turn.Back();
					Robot.Response(RB_TURNBACK);
				}
			}
			return;
		}
		
	}
}

uint32_t millis(void){
	return _millis_AMR;
}

ISR(INT0_vect){
	__captureA = TCNT1;
	if(MA.EN.state){
		MA.EN.DIR = -1;
		MA.EN.val--;
	}
	else{
		MA.EN.DIR = 1;
		MA.EN.val++;
	}
	if(__captureA < MA.EN.cap) MA.EN.pps = F_CPU/((__captureA + PWM_MAX - MA.EN.cap + 1) + (MA.EN.ovf_times-1)*(PWM_MAX+1));
	else MA.EN.pps = F_CPU/((__captureA - MA.EN.cap + 1) + MA.EN.ovf_times*(PWM_MAX+1));
	if(MA.EN.DIR == -1) MA.EN.pps =  -MA.EN.pps;
	MA.EN.cap = __captureA;	
	MA.EN.ovf_times = 0;	
	if(!MA.PID.state) MA.EN.pps = 0;
	MA.Mean_filt();
}

ISR(INT1_vect){
	__captureB = TCNT1;
	if(MB.EN.state){
		MB.EN.DIR = -1;
		MB.EN.val--;
	}
	else{
		MB.EN.DIR = 1;
		MB.EN.val++;
	}
	if(__captureB < MB.EN.cap) MB.EN.pps = F_CPU/((__captureB + PWM_MAX - MB.EN.cap + 1) + (MB.EN.ovf_times-1)*(PWM_MAX+1));
	else MB.EN.pps = F_CPU/((__captureB - MB.EN.cap + 1) + MB.EN.ovf_times*(PWM_MAX+1));
	if(MB.EN.DIR == -1) MB.EN.pps =  -MB.EN.pps;
	MB.EN.cap = __captureB;	
	MB.EN.ovf_times = 0;
	if(!MB.PID.state) MB.EN.pps = 0;
	MB.Mean_filt();	
}

ISR(INT2_vect){
	E_Stop();
}

ISR(TIMER1_OVF_vect){	
	Robot.PID_loop();
	Robot.Check_stop();	
	MA.EN.state = (PINA>>C2A)&1;
	MB.EN.state = (PINA>>C1B)&1;
}

ISR(TIMER0_COMP_vect){
	if(Robot.Behavior == ACCELERATE) Robot.Accelerate();
	else if(Robot.Behavior == UNI_MOTION) Robot.UniMotion();
	else if(Robot.Behavior == DECELERATE) Robot.Decelerate();
	ST_But = (PINB>>ST)&1;
	if(Check_Button(ST_But,L_ST_But)){
		Robot.SetSTART(1);
	}
	L_ST_But = ST_But;
	Led_Ind(START);
}

ISR(TIMER2_COMP_vect){
	_millis_AMR++;
	if((_millis_AMR - _ADCMillis) >= ADC_INTERVAL){
		_ADCMillis = _millis_AMR;
		ADC_Trig(BAT);
		buzzerControl();
	}			
}

ISR(ADC_vect){
	_currentADCValue = ADCW;
	percentBat = ADCtoPercent(_currentADCValue);
	if(percentBat != lastPercentBat){
		i_ADC++;
		if(i_ADC > 3){
			i_ADC = 0;
			if(percentBat < 15){
				E_Stop();
				Robot.Response(ERR_01);
			}	
			lastPercentBat = percentBat;
		}
	}
}


/*
			if(Robot.Done_bit == 'd'){
				Robot.Stop();
				if(Process < NoProcess) Process++;
				_delay_ms(1000);
				if(Process == 1) Robot.Forward.cm(100);
				if(Process == 2) Robot.Backward.cm(100);
			
			}
			*/

/*
	
// 	MB.MFilt.sum = 0;
// 	for(i_meanB = 0; i_meanB < Filt_order-1; i_meanB++){
// 		MB.MFilt.val[i_meanB] = MB.MFilt.val[i_meanB+1];
// 		MB.MFilt.sum = MB.MFilt.sum + MB.MFilt.val[i_meanB];
// 	}
// 	MB.MFilt.val[Filt_order - 1] = MB.EN.pps;
// 	MB.MFilt.sum = MB.MFilt.sum + MB.MFilt.val[Filt_order - 1];
// 	MB.MFilt.vec = MB.MFilt.sum/Filt_order;

	
// 	MA.MFilt.sum = 0;
// 	for(i_meanA = 0; i_meanA < Filt_order-1; i_meanA++){ 
// 		MA.MFilt.val[i_meanA] = MA.MFilt.val[i_meanA+1];
// 		MA.MFilt.sum = MA.MFilt.sum + MA.MFilt.val[i_meanA];
// 	}
// 	MA.MFilt.val[Filt_order - 1] = MA.EN.pps;
// 	MA.MFilt.sum = MA.MFilt.sum + MA.MFilt.val[Filt_order - 1];
// 	MA.MFilt.vec = MA.MFilt.sum/Filt_order;



// 	else if(Robot.Direction == TURN_RIGHT){
// 		if((MA.MFilt.vec > -SPEED_CONST) || (MB.MFilt.vec < SPEED_CONST)) {
// 			MA.PID.REF = MA.PID.REF - ACC_CONST;
// 			MB.PID.REF = MB.PID.REF + ACC_CONST;
// 		}
// 		else{
// 			MA.PID.REF = -SPEED_CONST;
// 			MB.PID.REF = SPEED_CONST;
// 			Robot.L_Behavior = Robot.Behavior;
// 			Robot.Behavior = UNI_MOTION;
// 			PID_TrigParameters(UNI_MOTION);
// 		}
// 	}
// 	else if(Robot.Direction == TURN_LEFT){
// 		if((MA.MFilt.vec < SPEED_CONST) || (MB.MFilt.vec > -SPEED_CONST)) {
// 			MA.PID.REF = MA.PID.REF + ACC_CONST;
// 			MB.PID.REF = MB.PID.REF - ACC_CONST;
// 		}
// 		else{
// 			MA.PID.REF = SPEED_CONST;
// 			MB.PID.REF = -SPEED_CONST;
// 			Robot.L_Behavior = Robot.Behavior;
// 			Robot.Behavior = UNI_MOTION;
// 			PID_TrigParameters(UNI_MOTION);
// 		}
// 	}



// 	else if(Robot.Direction == TURN_RIGHT){
// 		if((MA.EN.val < -Robot.dec_point) || (MB.EN.val > Robot.dec_point)){
// 			Robot.L_Behavior = Robot.Behavior;
// 			Robot.Behavior = DECELERATE;
// 			PID_TrigParameters(DECELERATE);
// 		}
// 	}
// 	else if(Robot.Direction == TURN_LEFT){
// 		if((MA.EN.val > Robot.dec_point) || (MB.EN.val < - Robot.dec_point)){
// 			Robot.L_Behavior = Robot.Behavior;
// 			Robot.Behavior = DECELERATE;
// 			PID_TrigParameters(DECELERATE);
// 		}
// 	}

// 	else if(Robot.Direction == TURN_RIGHT){
// 		if((MA.EN.val <= -Robot.duty) || (MB.EN.val >= Robot.duty)) {
// 			DecelerateDone();
// 		}
// 		else{
// 			MA.PID.REF = MA.PID.REF + DEC_CONST;
// 			MB.PID.REF = MB.PID.REF - DEC_CONST;
// 		}
// 	}
// 	else if(Robot.Direction == TURN_LEFT){
// 		if((MA.EN.val >= Robot.duty) || (MB.EN.val <= -Robot.duty)) {
// 			DecelerateDone();
// 		}
// 		else{
// 			MA.PID.REF = MA.PID.REF - DEC_CONST;
// 			MB.PID.REF = MB.PID.REF + DEC_CONST;
// 		}
// 	}


// 	if(_dir == TURN_RIGHT){
// 		Robot.Direction = TURN_RIGHT;
// 		Robot.duty = (WHEELS_DISTANCE*TURNR_COEF/(4*WHEELS_DIA))*ENCODER_TYPE;
// 		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
// 		PID_TrigParameters(ACCELERATE);
// 		PID_SetValues(-START_SPEED,START_SPEED);
// 		
// 	}
// 	else if(_dir == TURN_LEFT){
// 		Robot.Direction = TURN_LEFT;
// 		Robot.duty = (WHEELS_DISTANCE*TURNL_COEF/(4*WHEELS_DIA))*ENCODER_TYPE;
// 		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
// 		PID_TrigParameters(ACCELERATE);
// 		PID_SetValues(START_SPEED,-START_SPEED);
// 	}

// 	if((Robot.Behavior == ACCELERATE) || (Robot.Behavior == DECELERATE)){
// 		MB.MFilt.vec = MB.EN.pps;
// 	}
// 	else{
// 		MB.MFilt.vec = MB.MFilt.sum/Filt_order;
// 	}

// 	if((Robot.Behavior == ACCELERATE) || (Robot.Behavior == DECELERATE)){
// 		MA.MFilt.vec = MA.EN.pps;
// 	}
// 	else{
// 		MA.MFilt.vec = MA.MFilt.sum/Filt_order;
// 	}
*/

/*

void Mean_filt_MA_Clear(void){
	for(j = 0; j < Filt_order; j++){
		MA.MFilt.val[j] = 0;
	}
}

void Mean_filt_MB_Clear(void){
	for(j = 0; j < Filt_order; j++){
		MB.MFilt.val[j] = 0;
	}
}
*/