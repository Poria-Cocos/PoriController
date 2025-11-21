#ifndef __BUTTON_BSP_H
#define __BUTTON_BSP_H

#include "stm32g4xx_hal.h"

#define BUTTON_MAX_NUM 32

typedef struct {
	uint8_t SW0;
	uint8_t SW1;
	uint8_t SW2;
	uint8_t SW3;
	uint8_t SW4;
	uint8_t SW5;
	uint8_t SW6;
	uint8_t SW7;
	uint8_t SW8;
	uint8_t SW9;
	uint8_t SW10;
	uint8_t SW11;
	uint8_t Toggle1B;
	uint8_t Toggle1A;
	uint8_t Toggle2B;
	uint8_t Toggle2A;
	uint8_t Toggle3B;
	uint8_t Toggle3A;
	uint8_t Toggle4B;
	uint8_t Toggle4A;
	uint8_t EncoderLA;
	uint8_t EncoderLB;
	uint8_t EncoderLSW;
	uint8_t EncoderRA;
	uint8_t EncoderRB;
	uint8_t EncoderRSW;
	uint8_t RockerLSW;
	uint8_t RockerRSW;
} btn_t;

typedef struct {
	GPIO_TypeDef* port;
	uint16_t pin;
	uint8_t state;         // 当前稳定状态（0:松开, 1:按下）
	uint8_t last_state;    // 上一次稳定状态
	uint8_t debounce_cnt;  // 消抖计数
	uint8_t event_flag;    // 事件标志（1:有新事件）
	uint8_t used;          // 是否已注册
} button_t;

extern btn_t BTN;
extern button_t buttons[BUTTON_MAX_NUM];

// 注册一个按键，返回分配的索引（失败返回0xFF）
uint8_t Button_Register(GPIO_TypeDef* port, uint16_t pin);
// 初始化所有已注册按键
void Button_Init(void);
// 扫描（建议10~20ms调用一次）
void Button_Scan(void);
// 获取事件，返回1有事件，*pressed=1按下，0松开
uint8_t Button_GetEvent(uint8_t* idx_addr);

// 旋转编码器读取，返回0x01左旋，0x02右旋，0无旋转
uint8_t Encoder_Scan(GPIO_TypeDef* portA, uint16_t pinA, GPIO_TypeDef* portB, uint16_t pinB);

#endif
