
#include "MyRobot.h"

uint8_t i_meda, i_medb, i, j, START, ESTOP, ST_But, L_ST_But,R_Steer, L_Steer, severConnection = 0, speedReturn = 0, encoderReturn = 0;
uint16_t __captureA, __captureB;
uint32_t _millis_AMR = 0, _lastMillis = 0;

// char *buff = (char *) calloc(10, sizeof(char));
// char *buff_val = (char *) calloc(10, sizeof(char));
char buff[EVENT_BUFFER_SIZE];
char buff_val[EVENT_BUFFER_SIZE];
char pps_vals[18];

void IO_Init(void){
	DDRB |= (1<<SST) | (1<<SES);
	PORTB &= ~((1<<SST) | (1<<SES));
	DDRD |= (1<<EA) | (1<<EB);
	PORTD &= ~((1<<EA) | (1<<EB));
	DDRC |= (1<<LA) | (1<<RA) | (1<<LB) | (1<<RB);
	PORTC &= ~((1<<LA) | (1<<RA) | (1<<LB) | (1<<RB));
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
	OCR0 = 185;
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
		PORTD |= (1<<EA);
	}
}

void MBTrig(int8_t __DIR){
	MB.EN.DIR = __DIR;
	if(__DIR == 1){
		PORTC &= ~(1<<LB);
		PORTC |= (1<<RB);
	}
	else if(__DIR == -1){
		PORTC &= ~(1<<RB);
		PORTC |= (1<<LB);
	}
	else{
		PORTC &= ~((1<<LB) | (1<<RB));
		OCR1B = PWM_MAX;
		MB.EN.pps = 0;
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
	MA.PID.last_er = 0;
	MA.PID.sum_er = 0;
	MA.PID.dif_er = 0;
	MA.PID.frame = 0;
	MA.EN.ovf_times = 0;
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
	MB.PID.last_er = 0;
	MB.PID.sum_er = 0;
	MB.PID.dif_er = 0;
	MB.PID.frame = 3;
	MB.EN.ovf_times = 0;
	MB.PID.state = 1;
}

void PID_SetValues(float __rpmA, float __rpmB){
	PID_A_SetValue(__rpmA);
	PID_B_SetValue(__rpmB);
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
		else OCR1A = abs(MA.PID.er_control);
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
		else OCR1B = abs(MB.PID.er_control);
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
	if(MA.EN.ovf_times >= 300){
		MA.EN.pps = 0;
		MA.EN.ovf_times = 0;
		MA.EN.DIR = 0;
		MA.Mean_filt();
	}
	if(MB.EN.ovf_times >= 300){
		MB.EN.pps = 0;
		MB.EN.ovf_times = 0;
		MB.EN.DIR = 0;
		MB.Mean_filt();
	}
}

void Mean_filt_MA(void){
	MA.MFilt.sum = 0;
	for(i_meda = 0; i_meda < Filt_order-1; i_meda++){
		MA.MFilt.val[i_meda] = MA.MFilt.val[i_meda+1];
		MA.MFilt.sum = MA.MFilt.sum + MA.MFilt.val[i_meda];
	}
	MA.MFilt.val[Filt_order - 1] = MA.EN.pps;
	MA.MFilt.sum = MA.MFilt.sum + MA.MFilt.val[Filt_order - 1];
	MA.MFilt.vec = MA.MFilt.sum/Filt_order;
}

void Mean_filt_MB(void){
	MB.MFilt.sum = 0;
	for(i_medb = 0; i_medb < Filt_order-1; i_medb++){
		MB.MFilt.val[i_medb] = MB.MFilt.val[i_medb+1];
		MB.MFilt.sum = MB.MFilt.sum + MB.MFilt.val[i_medb];
	}
	MB.MFilt.val[Filt_order - 1] = MB.EN.pps;
	MB.MFilt.sum = MB.MFilt.sum + MB.MFilt.val[Filt_order - 1];
	MB.MFilt.vec = MB.MFilt.sum/Filt_order;
}

void Stop(void){
	PORTC &= ~((1<<LA) | (1<<RA) | (1<<LB) | (1<<RB));
	MA.PID.state = 0;
	MB.PID.state = 0;
	MBTrig(0);
	MATrig(0);
	_delay_ms(20);
	MBTrig(0);
	MATrig(0);
	Robot.L_Behavior = Robot.Behavior;
	Robot.Behavior = 's';
	Robot.Direction = 's';
	Robot.Done_bit = 'd';
}	

void E_Stop(void){
	DecelerateDone();
	ESTOP = 1;
	START = 0;
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
	START = __start&1;
	ESTOP = (~__start)&1;
	Robot.Done_bit = 'd';
	UART.Clear_Buffer();
}

uint8_t Robot_GetESTOP(void){
	return ESTOP;
}

void DecelerateDone(void){
	Stop();
	Stop();
	_delay_ms(50);
	Robot.Done_bit = 'd';
}

void Robot_Accelerate(void){
	if(Robot.Direction == 'f'){
		if((MA.MFilt.vec < SPEED_CONST) && (MB.MFilt.vec < SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF + ACC_CONST;
			MB.PID.REF = MB.PID.REF + ACC_CONST;
		}
		else{
			MA.PID.REF = SPEED_CONST;
			MB.PID.REF = SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'c';
		}
	}
	else if(Robot.Direction == 'b'){
		if((MA.MFilt.vec > -SPEED_CONST) && (MB.MFilt.vec > -SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF - ACC_CONST;
			MB.PID.REF = MB.PID.REF - ACC_CONST;
		}
		else{
			MA.PID.REF = -SPEED_CONST;
			MB.PID.REF = -SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'c';
		}
	}
	else if(Robot.Direction == 'r'){
		if((MA.MFilt.vec > -SPEED_CONST) || (MB.MFilt.vec < SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF - ACC_CONST;
			MB.PID.REF = MB.PID.REF + ACC_CONST;
		}
		else{
			MA.PID.REF = -SPEED_CONST;
			MB.PID.REF = SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'c';
		}
	}
	else if(Robot.Direction == 'l'){
		if((MA.MFilt.vec < SPEED_CONST) || (MB.MFilt.vec > -SPEED_CONST)) {
			MA.PID.REF = MA.PID.REF + ACC_CONST;
			MB.PID.REF = MB.PID.REF - ACC_CONST;
		}
		else{
			MA.PID.REF = SPEED_CONST;
			MB.PID.REF = -SPEED_CONST;
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'c';
		}
	}

}

void Robot_UniMotion(void){
	if(Robot.Direction == 'f'){
		if((MA.EN.val > Robot.dec_point) || (MB.EN.val > Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'd';
			R_Steer = 0;
			L_Steer = 0;
		}
		if(R_Steer){
			if(MB.PID.REF > MA.PID.REF){
				MB.PID.REF = MB.PID.REF - STEER_INC;
				MA.PID.REF = MA.PID.REF + STEER_INC;
			}
			else{
				MA.PID.REF = SPEED_CONST;
				MB.PID.REF = MA.PID.REF;
				R_Steer = 0;
			}
		}
		if(L_Steer){
			if(MA.PID.REF > MB.PID.REF){
				MA.PID.REF = MA.PID.REF - STEER_INC;
				MB.PID.REF = MB.PID.REF + STEER_INC;
			}
			else{
				MB.PID.REF = SPEED_CONST;
				MA.PID.REF = MB.PID.REF;
				L_Steer = 0;
			}
		}
	}
	else if(Robot.Direction == 'b'){
		if((MA.EN.val < Robot.dec_point) || (MB.EN.val < Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'd';
			R_Steer = 0;
			L_Steer = 0;
		}
	}
	else if(Robot.Direction == 'r'){
		if((MA.EN.val < -Robot.dec_point) || (MB.EN.val > Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'd';
		}
	}
	else if(Robot.Direction == 'l'){
		if((MA.EN.val > Robot.dec_point) || (MB.EN.val < - Robot.dec_point)){
			Robot.L_Behavior = Robot.Behavior;
			Robot.Behavior = 'd';
		}
	}
	
}

void Robot_Decelerate(void){
	if(Robot.Direction == 'f'){
		if((MA.EN.val >= Robot.duty) || (MB.EN.val >= Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF - 1;
			MB.PID.REF = MB.PID.REF - 1;
		}
	}
	else if(Robot.Direction == 'b'){
		if((MA.EN.val <= Robot.duty) || (MB.EN.val <= Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF + 1;
			MB.PID.REF = MB.PID.REF + 1;
		}
	}
	else if(Robot.Direction == 'r'){
		if((MA.EN.val <= -Robot.duty) || (MB.EN.val >= Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF + 0.8;
			MB.PID.REF = MB.PID.REF - 0.8;
		}
	}
	else if(Robot.Direction == 'l'){
		if((MA.EN.val >= Robot.duty) || (MB.EN.val <= -Robot.duty)) {
			DecelerateDone();
		}
		else{
			MA.PID.REF = MA.PID.REF - 0.8;
			MB.PID.REF = MB.PID.REF + 0.8;
		}
	}	
	
}

void Robot_Forward_cm(float _cm){
	if((_cm > MAX_RANGE) || (_cm < 0)) return;
	Robot.duty = (_cm/(PI*WHEELS_DIA))*ENCODER_TYPE;
	Robot.dec_point = Robot.duty - DECELERATE_PT;
	Robot.Direction = 'f';
	Robot.Behavior = 'a';
	Robot.L_Behavior = 's';
	Robot.Done_bit = 'n';
	MA.EN.val = 0;
	MB.EN.val = 0;
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
	if((_cm > MAX_RANGE) || (_cm < 0)) return;
	Robot.duty = -(_cm/(PI*WHEELS_DIA))*ENCODER_TYPE;
	Robot.dec_point = Robot.duty + DECELERATE_PT;
	Robot.Direction = 'b';
	Robot.Behavior = 'a';
	Robot.L_Behavior = 's';
	Robot.Done_bit = 'n';
	MA.EN.val = 0;
	MB.EN.val = 0;
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
	if(Robot.Behavior == 'c') {
		R_Steer = 1;
		if(Robot.Direction == 'f'){
			MB.PID.REF = MB.PID.REF*STEER_COEF;
			MA.PID.REF = MA.PID.REF/STEER_COEF;
		}
	}
}

void Robot_LeftSteer(void){
	if(Robot.Behavior == 'c') {
		L_Steer = 1;
		if(Robot.Direction == 'f'){
			MA.PID.REF = MA.PID.REF*STEER_COEF;
			MB.PID.REF = MB.PID.REF/STEER_COEF;
		}
	}
}

void Robot_SetUpTurn(uint8_t _dir){
	Robot.Behavior = 'a';
	Robot.L_Behavior = 's';
	Robot.Done_bit = 'n';
	MA.EN.val = 0;
	MB.EN.val = 0;
	if(_dir == TURN_RIGHT){
		Robot.Direction = 'r';
		Robot.duty = (WHEELS_DISTANCE*TURNR_COEF/(4*WHEELS_DIA))*ENCODER_TYPE;
		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
		PID_SetValues(-START_SPEED,START_SPEED);
	}
	else if(_dir == TURN_LEFT){
		Robot.Direction = 'l';
		Robot.duty = (WHEELS_DISTANCE*TURNL_COEF/(4*WHEELS_DIA))*ENCODER_TYPE;
		Robot.dec_point = Robot.duty - DECELERATE_PTLR;
		PID_SetValues(START_SPEED,-START_SPEED);
	}
}

void Robot_Turn_Right(void){
	if(Robot.Behavior == 's'){
		Robot_SetUpTurn(TURN_RIGHT);
	}
}

void Robot_Turn_Left(void){
	if(Robot.Behavior == 's'){
		Robot_SetUpTurn(TURN_LEFT);
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
	Robot.GetST_Button = &Robot_GetST_Button;
	Robot.SetSTART = &Robot_SetSTART;
	Robot.GetESTOP = &Robot_GetESTOP;
	Robot.Forward.cm = &Robot_Forward_cm;
	Robot.Forward.m = &Robot_Forward_m;
	Robot.Forward.rounds = &Robot_Forward_r;
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
	Robot.GetConnection = &Robot_GetConnection;
	Robot.Response = &Robot_Response;
	Robot.EventHandler = &Robot_EventHandler;
	Robot.Direction = 's';
	Robot.Behavior = 's';
	
	Ex_INT_Init();
	Fast_PWM_20KHz_Init();
	Timer0_CTC_Init();
	Timer2_CTC_Init();
	UART_Init(9600);
	IO_Init();
	E_Stop();
	_delay_ms(200);
	I2C_LCD_Init(I2C_FREQ,LS_NONE);
	sei();
	LCD.Clear();
	LCD.WriteString("Initialized...");
	Robot_Response(RB_INITIALIZE);	
	_delay_ms(200);
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
	buff_val[0] = buff[2];
	for(j = 0; j < 4; j++){
		if(!isdigit(buff[3 + j])) break;
		buff_val[j+1] = buff[3 + j];	
	}		
	buff_val[j+1] = '\0';		
	speed = atoi(buff_val);

	if(_motor == MOTOR_A){
		PID_A_SetValue(speed);
		Robot_Response(RB_PID_A);
	}
	else if(_motor == MOTOR_B){
		PID_B_SetValue(speed);
		Robot_Response(RB_PID_B);
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
		UART.TX.String("AMR.av\n");
		return;
	}
	else if(_case == RB_DISCONNECTED){
//		UART.TX.String("Disconnected to server\n");
		UART.TX.String("AMR.uav\n");	
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
	else if(_case == RB_SPEED){
		if(speedReturn){
			sprintf(pps_vals,"%6d,%6d;\n",((int16_t)MA.MFilt.vec), ((int16_t)MB.MFilt.vec));
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
		Robot.Done_bit = 'r';
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
}

void Robot_EventHandler(void){
	if(Robot.GetESTOP()){
		if((_millis_AMR - _lastMillis) > ES_UPDATE_INTERVAL){
			_lastMillis = _millis_AMR;
			Robot_Response(RB_EMERGENCY_STOP);
		}	
	}
	if(UART.RX.String_Done()){
		Robot_EventBufferClear();
		UART.RX.String(buff,EVENT_BUFFER_SIZE);
		if((buff[0] == 'e') && (buff[1] == 's')){
			Robot.EStop();
			_delay_ms(100);
			while(!(Robot.Done_bit == 'd'));
			Robot_Response(RB_EMERGENCY_STOP);
		}
		else if((buff[0] == 'c') && (buff[1] == 'n') && (buff[2] == 't')){
			severConnection = 1;
			Robot_Response(RB_CONNECTED);
		}
		else if((buff[0] == 'd') && (buff[1] == 'n') && (buff[2] == 't')){
			severConnection = 0;
			Robot_Response(RB_DISCONNECTED);
		}
		else if((buff[0] == 'g') && (buff[1] == 'g')){
			Robot.Stop();
			_delay_ms(100);
			while(!(Robot.Done_bit == 'd'));
			Robot_Response(RB_TEMPORARILY_STOP);

		}
		else if((buff[0] == 'g') && (buff[1] == 'o')){
			Robot.SetSTART(1);
			while(!(Robot.Done_bit == 'd'));
			Robot_Response(RB_SET_START);
		}
		else if((buff[0] == 'c') && (buff[1] == 'm') && (buff[2] == 'd') && (buff[3] == '0')){
			if(buff[4] == '1'){
				speedReturn = 1;
				Robot_Response(CMD_01);
			}
			else if(buff[4] == '2'){
				speedReturn = 0;
				Robot_Response(CMD_02);
			}
			else if(buff[4] == '3'){
				encoderReturn = 1;
				Robot_Response(CMD_03);
			}
			else if(buff[4] == '4'){
				encoderReturn = 0;
				Robot_Response(CMD_04);
			}
		}
		if(Robot.GetESTOP()) return;
		if((buff[0] == 'P') && (buff[1] == 'A')){
			setPIDnResponse(MOTOR_A);
		}
		else if((buff[0] == 'P') && (buff[1] == 'B')){
			setPIDnResponse(MOTOR_B);
		}
		if(Robot.Done_bit == 'r'){
			uint16_t distance = 0;
			if(buff[0] == 'f'){
				for(j = 1; j < EVENT_BUFFER_SIZE; j++){
					if(!isdigit(buff[j])) break;
					buff_val[j-1] = buff[j];	
				}		
				buff_val[j-1] = '\0';		
				distance = atoi(buff_val);
 				Robot.Forward.cm(distance);
				Robot_Response(RB_FORWARD);
			}
			else if(buff[0] == 'b'){
				for(j = 1; j < EVENT_BUFFER_SIZE; j++){
					if(!isdigit(buff[j])) break;
					buff_val[j-1] = buff[j];	
				}				
				buff_val[j-1] = '\0';
				distance = atoi(buff_val);
				Robot.Backward.cm(distance);
				Robot_Response(RB_BACKWARD);
			}
			else if(buff[0] == 't'){
				if(buff[1] == 'r'){
					Robot.Turn.Right();
					Robot_Response(RB_TURNRIGHT);
				}
				if(buff[1] == 'l'){
					Robot.Turn.Left();
					Robot_Response(RB_TURNLEFT);
				}
			}
		}
		else if(Robot.Done_bit == 'n'){
			if(buff[0] == 's'){
				if(buff[1] == 'r') {
					Robot.Steer.Right();
					Robot_Response(RB_STEERRIGHT);
				}
				if(buff[1] == 'l'){
					Robot.Steer.Left();
					Robot_Response(RB_STEERLEFT);
				}
			}
		}
	}
}

uint32_t millis(void){
	return _millis_AMR;
}

ISR(INT0_vect){
	__captureA = TCNT1;
	if(MA.EN.state){
		MA.EN.DIR = 1;
		MA.EN.val++;
	}
	else{
		MA.EN.DIR = -1;
		MA.EN.val--;
	}
	if(__captureA < MA.EN.cap) MA.EN.pps = F_CPU/((__captureA + PWM_MAX - MA.EN.cap + 1) + (MA.EN.ovf_times-1)*(PWM_MAX+1));
	else MA.EN.pps = F_CPU/((__captureA - MA.EN.cap + 1) + MA.EN.ovf_times*(PWM_MAX+1));
	if(MA.EN.DIR == -1) MA.EN.pps =  -MA.EN.pps;
	MA.EN.cap = __captureA;	
	MA.EN.ovf_times = 0;	
	MA.Mean_filt();
}

ISR(INT1_vect){
	__captureB = TCNT1;
	if(MB.EN.state){
		MB.EN.DIR = 1;
		MB.EN.val++;
	}
	else{
		MB.EN.DIR = -1;
		MB.EN.val--;
	}
	if(__captureB < MB.EN.cap) MB.EN.pps = F_CPU/((__captureB + PWM_MAX - MB.EN.cap + 1) + (MB.EN.ovf_times-1)*(PWM_MAX+1));
	else MB.EN.pps = F_CPU/((__captureB - MB.EN.cap + 1) + MB.EN.ovf_times*(PWM_MAX+1));
	if(MB.EN.DIR == -1) MB.EN.pps =  -MB.EN.pps;
	MB.EN.cap = __captureB;	
	MB.EN.ovf_times = 0;
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
	if(Robot.Behavior == 'a') Robot.Accelerate();
	else if(Robot.Behavior == 'c') Robot.UniMotion();
	else if(Robot.Behavior == 'd') Robot.Decelerate();
	ST_But = (PINB>>ST)&1;
	if(Check_Button(ST_But,L_ST_But)){
		Robot.SetSTART(1);
	}
	L_ST_But = ST_But;
	Led_Ind(START);
}

ISR(TIMER2_COMP_vect){
	_millis_AMR++;
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
