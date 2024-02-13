#include "main.h"
#include "button.h"

extern void delay_us(unsigned long us);
extern int dotmatrix_main(int up_down_status);
extern void i2c_lcd_main(void);
extern int destination_floor;
extern int now_floor;


void stepmotor_main(void);
void stop_motor(void);
void stepmotor_drive(int derection);
void set_rpm(int rpm);

void set_rpm(int rpm)  // rpm 1~13
{
	delay_us(60000000 / 4096 / rpm);
	// 최대 speed 기준(13) : delay_us(1126)
}

int first = 1;
int second = 0;
int third = 0;
int fourth = 0;

int forward = 0;
int stop = 1;
int backward = 0;

int cnt = 0;

void stop_motor(void)
{
	cnt = 0;
	stop = 1;
	forward = 0;
	backward = 0;
}

void stepmotor_main(void)  // 현재 상태를 기준으로 작성 -> 원래의 get button을 기준으로 작성해보기
{
	if (get_button(BUTTON3_GPIO_Port, BUTTON3_Pin, 3) == BUTTON_PRESS)
	{
		destination_floor++;
		destination_floor%=5;

		if (destination_floor == 0)
		{
			destination_floor = 1;
		}
		i2c_lcd_main();
	}
	if (stop)
	{
		if (get_button(BUTTON2_GPIO_Port, BUTTON2_Pin, 2) == BUTTON_PRESS)  // Increase
		{
			forward = 1;
			stop = 0;
			cnt = 1;
		}
		if (get_button(BUTTON0_GPIO_Port, BUTTON0_Pin, 0) == BUTTON_PRESS)  // Decrease
		{
			backward = 1;
			stop = 0;
			cnt = 2;
		}
	}
	if (forward == 1)
	{
		if (get_button(BUTTON1_GPIO_Port, BUTTON1_Pin, 1) == BUTTON_PRESS)  // Stop
		{
			stop = 1;
			forward = 0;
			cnt = 0;
		}
		if (get_button(BUTTON0_GPIO_Port, BUTTON0_Pin, 0) == BUTTON_PRESS)  // Decrease
		{
			backward = 1;
			forward = 0;
			cnt = 2;
		}
	}
	if (backward == 1)
	{
		if (get_button(BUTTON2_GPIO_Port, BUTTON2_Pin, 2) == BUTTON_PRESS)  // Increase
		{
			forward = 1;
			backward = 0;
			cnt = 1;
		}
		if (get_button(BUTTON1_GPIO_Port, BUTTON1_Pin, 1) == BUTTON_PRESS)  // Stop
		{
			stop = 1;
			backward = 0;
			cnt = 0;
		}
	}

	stepmotor_drive(cnt);  // cnt랑 겹치는 변수가 존재 > 하나로 합치기

	if (destination_floor == now_floor)
	{
		stop = 1;
		forward = 0;
		backward = 0;
		cnt = 0;
	}
}

/*
void elevator_main(void)
{
	int cnt = 0;  // 상태를 나타낼 변수
	while(1)
	{
		if (get_button(BUTTON1_GPIO_Port, BUTTON1_Pin, 1) == BUTTON_PRESS)  // Increase
		{
			cnt=1;
			//dotmatrix_main(1);
		}
		if (get_button(BUTTON0_GPIO_Port, BUTTON0_Pin, 0) == BUTTON_PRESS)  // Decrease
		{
			cnt=2;
			//dotmatrix_main(2);
		}
		if (get_button(BUTTON2_GPIO_Port, BUTTON2_Pin, 2) == BUTTON_PRESS)  // Stop
		{
			cnt=3;
			//dotmatrix_main(3);
		}
		stepmotor_drive(cnt);
		//i2c_lcd_main();
	}
}
*/
void stepmotor_drive(int derection)
{
	static int step = 0;  // 회전을 위한 변수

	if (derection == 1)  // 정회전/상승
		{
			step++;  // for(step=0; step<8; step++)
			step %= 8;  // 다음 진행할 step 준비
			set_rpm(13);
		}
	else if (derection == 2)  // 역회전/하강
	{
		step--;
		if (step < 0) step = 7;
		set_rpm(13);
	}
	else if (derection == 0)
	{
		HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
		return;
	}

	switch(step){
		case 0:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 0);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 1);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 1);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIOD, IN1_Pin, 1);
			HAL_GPIO_WritePin(GPIOD, IN2_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN3_Pin, 0);
			HAL_GPIO_WritePin(GPIOD, IN4_Pin, 1);
			break;
		default:
			break;
	}
}
