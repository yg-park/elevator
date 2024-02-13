#include "main.h"

extern int forward;
extern int stop;
extern int backward;

void dotmatrix_main(void);
void init_dotmatrix(void);

GPIO_TypeDef *col_port[] =
{
	COL1_GPIO_Port, COL2_GPIO_Port, COL3_GPIO_Port, COL4_GPIO_Port,
	COL5_GPIO_Port, COL6_GPIO_Port, COL7_GPIO_Port, COL8_GPIO_Port
};

GPIO_TypeDef *row_port[] =
{
	ROW1_GPIO_Port, ROW2_GPIO_Port, ROW3_GPIO_Port, ROW4_GPIO_Port,
	ROW5_GPIO_Port, ROW6_GPIO_Port, ROW7_GPIO_Port, ROW8_GPIO_Port
};

uint16_t row_pin[] =
{
	ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin,
	ROW5_Pin, ROW6_Pin, ROW7_Pin, ROW8_Pin
};

uint16_t col_pin[] =
{
	COL1_Pin, COL2_Pin, COL3_Pin, COL4_Pin,
	COL5_Pin, COL6_Pin, COL7_Pin, COL8_Pin
};

const uint8_t _down[8] = {	  // ->
	0b00001000,
	0b00011000,
	0b00111110,
	0b01111110,
	0b01111110,
	0b00111110,
	0b00011000,
	0b00001000
};

const uint8_t _up[8] = {	  // <-
	0b00010000,
	0b00011000,
	0b01111100,
	0b01111110,
	0b01111110,
	0b01111100,
	0b00011000,
	0b00010000
};

const uint8_t _stop[8] = {    // hart
	0b00011110,
	0b00111111,
	0b01111111,
	0b11111110,
	0b11111110,
	0b01111111,
	0b00111111,
	0b00011110
	};

unsigned char display_data[8];  // 최종 8x8 출력할 데이터
unsigned char scroll_buffer[10][8] = {0}; // 스크롤할 데이터가 들어있는 버퍼, unsigned로 하는 이유는 숫자가 아니라 데이터로 쓰겠다는 의미

int number_of_character = 1;  // 출력할 문자 개수

// 초기화 작업
// 1. display_data에 number_data[0]에 있는 내용을 복사
// 2. number_data를 scroll_buffer에 복사
// 3. dotmatrix의 led를 off

void init_dotmatrix_up(void)
{
	for (int i = 0; i < 8; i++)
	{
		display_data[i] = _up[i];
	}
	for (int i = 1; i < number_of_character + 1; i++)
	{
		for (int j = 0; j < 8; j++)  // scroll_buffer[0] = blank
		{
			scroll_buffer[i][j] = _up[j];
		}
	}
	for (int i = 0; i < 8; ++i)
	{
		HAL_GPIO_WritePin(col_port[i], col_pin[i], 1);
	}
}
void init_dotmatrix_down(void)
{
	for (int i = 0; i < 8; i++)
	{
		display_data[i] = _down[i];
	}
	for (int i = 1; i < number_of_character + 1; i++)
	{
		for (int j = 0; j < 8; j++)  // scroll_buffer[0] = blank
		{
			scroll_buffer[i][j] = _down[j];
		}
	}
	for (int i = 0; i < 8; ++i)
	{
		HAL_GPIO_WritePin(col_port[i], col_pin[i], 1);
	}
}
void init_dotmatrix_stop(void)
{
	for (int i = 0; i < 8; i++)
	{
		display_data[i] = _stop[i];
	}
	for (int i = 1; i < number_of_character + 1; i++)
	{
		for (int j = 0; j < 8; j++)  // scroll_buffer[0] = blank
		{
			scroll_buffer[i][j] = _stop[j];
		}
	}
	for (int i = 0; i < 8; ++i)
	{
		HAL_GPIO_WritePin(col_port[i], col_pin[i], 1);
	}
}


void write_column_data(int col)
{
	for	(int i=0; i<8; i++)
	{
		if (i==col)
		{
			HAL_GPIO_WritePin(col_port[i], col_pin[i], 0);  // on
		}
		else
		{
			HAL_GPIO_WritePin(col_port[i], col_pin[i], 1);  // off
		}
	}
}

// oboo111100
void write_row_data(unsigned char data)
{
	unsigned char d;

	d=data;
	for (int i=0; i<8; i++)
	{
		if (d & (1 << i))  // 1인경우
		{
			HAL_GPIO_WritePin(row_port[i], row_pin[i], 1);
		}
		else
		{
			HAL_GPIO_WritePin(row_port[i], row_pin[i], 0);
		}
	}
}

#if 1
// scroll 문자 출력 프로그램
void dotmatrix_main(void)
{
	static int count = 0; // column count
	static int index = 0; // scroll_buffer의 2차원 index 값
	static uint32_t past_time = 0;  // 이전 tick 값 저장

	if (stop == 1)
	{
		for (int i=0; i<8; i++)
		{
			// 공통 양극 방식
			// column에는 0을, row에는 1을 출력해야 해당 LED가 on 된다.
			write_column_data(i);
			write_row_data(_stop[i]);
			HAL_Delay(1);
		}
	}

	if (forward == 1)
	{
		uint32_t now = HAL_GetTick();  // 1ms
		// 처음 시작 시 past_time = 0; now=500 > past_time=500
		if (now-past_time >= 200)  // 500ms 마다 scroll
		{
			past_time = now;
			for (int i = 0; i < 8; ++i)
			{
				display_data[i] = (_up[i] >> count) | (_up[i] << (8 - count));
			}
			if (++count == 8)  // 8column을 다 처리 했으면 다음 scroll_buffer로 이동
			{
				count = 0;
			}
		}
		for (int i=0; i<8; i++)
		{
			// 공통 양극 방식
			// column에는 0을, row에는 1을 출력해야 해당 LED가 on 된다.
			write_column_data(i);
			write_row_data(display_data[i]);
			HAL_Delay(1);
		}
	}

	if (backward == 1)
	{
		uint32_t now = HAL_GetTick();  // 1ms
		// 처음 시작 시 past_time = 0; now=500 > past_time=500
		if (now-past_time >= 200)  // 500ms 마다 scroll
		{
			past_time = now;
			for (int i = 0; i < 8; ++i)
			{
				display_data[i] = (_down[i] << count) | (_down[i] >> (8 - count));
			}
			if (++count == 8)  // 8column을 다 처리 했으면 다음 scroll_buffer로 이동
			{
				count = 0;
			}
		}
		for (int i=0; i<8; i++)
		{
			// 공통 양극 방식
			// column에는 0을, row에는 1을 출력해야 해당 LED가 on 된다.
			write_column_data(i);
			write_row_data(display_data[i]);
			HAL_Delay(1);
		}
	}
}
#else
// 고정 문자 출력 demo program
int dotmatrix_main(void)
{
	for (int i=0; i<8; i++)
	{
		// 공통 양극 방식
		// column에는 0을, row에는 1을 출력해야 해당 LED가 on 된다.
		write_column_data(i);
		write_row_data(all_on[i]);
		HAL_Delay(1);
	}
	return 0;
}
#endif
