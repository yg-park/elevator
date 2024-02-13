#include "main.h"
#include "fnd4digit.h"
#include "button.h"

extern volatile int fnd1ms_counter;  // fnd1ms
extern int now_floor;

uint32_t FND_font[10] =
{
  FND_a|FND_b|FND_c|FND_d|FND_e|FND_f,   // 0
  FND_b|FND_c,                           // 1
  FND_a|FND_b|FND_d|FND_e|FND_g,         // 2
  FND_a|FND_b|FND_c|FND_d|FND_g,         // 3
  FND_b|FND_c|FND_f|FND_g,   // 4
  FND_a|FND_c|FND_d|FND_f|FND_g,  // 5
  FND_a|FND_c|FND_d|FND_e|FND_f|FND_g,  // 6
  FND_a|FND_b|FND_c,      // 7
  FND_a|FND_b|FND_c|FND_d|FND_e|FND_f|FND_g,   // 8
  FND_a|FND_b|FND_c|FND_d|FND_f|FND_g   // 9
};

void fnd1digit_main(void)
{
	FND1digit_off();

	HAL_GPIO_WritePin(FND_DATA_PORT, FND_font[now_floor], GPIO_PIN_RESET);
}


void FND1digit_off(void)
{
	HAL_GPIO_WritePin(FND_DATA_PORT,FND_font[8]|FND_p, GPIO_PIN_SET);
}
