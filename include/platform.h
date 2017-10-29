#pragma once

#include <stdbool.h>
#include <libopencm3/stm32/gpio.h>

#define KNOB_COUNT 4

enum Led
{
	LED_RED,
	LED_GREEN,
	LED_BLUE
};

typedef struct
{
	uint8_t analog : 1;
	uint8_t pullup : 1;
} KnobConfig;

void platformInit(const KnobConfig* knobConfig);
void platformMainloop(void);
void platformRegisterIdleCallback(void(*cb)(void));

static inline void setLed(enum Led led, bool state)
{
	switch (led)
	{
	case LED_GREEN:
		GPIO_BSRR(GPIOD) = state ? GPIO12 : (GPIO12 << 16);
		break;
	case LED_RED:
		GPIO_BSRR(GPIOD) = state ? GPIO14 : (GPIO14 << 16);
		break;
	case LED_BLUE:
		GPIO_BSRR(GPIOD) = state ? GPIO15 : (GPIO15 << 16);
		break;
	}
}

uint16_t knob(uint8_t n);
bool button(uint8_t n);
void platformFrameFinishedCB(void);
