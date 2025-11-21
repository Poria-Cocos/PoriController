#include "bsp_button.h"
#include "main.h"

// 按键消抖时间（单位：扫描周期数）
#define BUTTON_DEBOUNCE_TICKS 3

button_t buttons[BUTTON_MAX_NUM];

btn_t BTN;

uint8_t Button_Register(GPIO_TypeDef* port, uint16_t pin) {
	for (uint8_t i = 0; i < BUTTON_MAX_NUM; ++i) {
		if (!buttons[i].used) {
			buttons[i].port = port;
			buttons[i].pin = pin;
			buttons[i].used = 1;
			// 其余成员在init时初始化
			return i; // 返回分配的索引
		}
	}
	return 0xFF; // 注册失败
}

void Button_Init(void) {
	for (int i = 0; i < BUTTON_MAX_NUM; ++i) {
		if (buttons[i].used) {
			buttons[i].state = 1;
			buttons[i].last_state = 1;
			buttons[i].debounce_cnt = 0;
			buttons[i].event_flag = 0;
		}
	}
}

// 每隔10~20ms调用一次
void Button_Scan(void) {
	for (int i = 0; i < BUTTON_MAX_NUM; ++i) {
		if (!buttons[i].used) continue;
		buttons[i].state = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
	}
}

// 查询按键事件（返回1:有新事件且按下）
uint8_t Button_GetEvent(uint8_t* idx_addr) {
	uint8_t idx = *idx_addr;
	if (idx >= BUTTON_MAX_NUM || !buttons[idx].used) return 0;
	if(buttons[idx].state == 0){
		return 1;
	}
	return 0;
}

uint8_t Encoder_Scan(GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB) {
	static char lock = 0;
	static uint16_t knob_dir = 0;

	uint8_t gpio1 = HAL_GPIO_ReadPin(portA, pinA);
	uint8_t gpio2 = HAL_GPIO_ReadPin(portB, pinB);

	if ((gpio1 == 0 && gpio2 == 1) && (!lock)) {
		lock = 1;
		knob_dir = 1;
	}
	else if ((gpio2 == 0 && gpio1 == 1) && (!lock)) {
		lock = 1;
		knob_dir = 2;
	}
	else if ((gpio2 == 1 && gpio1 == 1)) {
		lock = 0;
		knob_dir = 0;
	}

	return knob_dir;
}
