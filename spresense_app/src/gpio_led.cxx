#include <arch/board/board.h>

#include "include/gpio_led.h"


#define PIN_LED0 PIN_I2S1_BCK
#define PIN_LED1 PIN_I2S1_LRCK
#define PIN_LED2 PIN_I2S1_DATA_IN
#define PIN_LED3 PIN_I2S1_DATA_OUT
#define PIN_BUTTON PIN_SPI3_CS1_X


volatile bool g_start_requested = false;

static int button_handler(int irq, void *context, void *arg) {
    g_start_requested = !g_start_requested;
    return OK;
}

void init_leds(void) {
    board_gpio_write(PIN_LED0, -1);
    board_gpio_config(PIN_LED0, 0, false, true, PIN_FLOAT);
    board_gpio_write(PIN_LED1, -1);
    board_gpio_config(PIN_LED1, 0, false, true, PIN_FLOAT);
    board_gpio_write(PIN_LED2, -1);
    board_gpio_config(PIN_LED2, 0, false, true, PIN_FLOAT);
    board_gpio_write(PIN_LED3, -1);
    board_gpio_config(PIN_LED3, 0, false, true, PIN_FLOAT);
}

void set_leds(int ptn){
    board_gpio_write(PIN_LED0, (ptn & 0x01) ? 1 : 0);
    board_gpio_write(PIN_LED1, (ptn & 0x02) ? 1 : 0);
    board_gpio_write(PIN_LED2, (ptn & 0x04) ? 1 : 0);
    board_gpio_write(PIN_LED3, (ptn & 0x08) ? 1 : 0);
}

void init_gpio(void) {
    board_gpio_config(PIN_BUTTON, 0, true, false, PIN_PULLUP);
    board_gpio_intconfig(PIN_BUTTON, INT_FALLING_EDGE, true, button_handler);
    board_gpio_int(PIN_BUTTON, true); // enable interrupt
}
