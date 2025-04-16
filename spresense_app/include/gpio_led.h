#ifndef GPIO_LED_H
#define GPIO_LED_H

extern volatile bool g_start_requested;

void init_leds(void);
void init_gpio(void);
void set_leds(int ptn);

#endif // GPIO_LED_H
