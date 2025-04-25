/**
 * A demonstration program that flashes some firmware into an RP2040 via 
 * the SWD port.
 * 
 * Copyright (C) Bruce MacKinnon, 2025
 */
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/flash.h"
#include "pico/bootrom.h"

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "kc1fsz-tools/rp2040/PicoClock.h"
#include "kc1fsz-tools/rp2040/PicoPollTimer.h"
#include "SX1276Driver.h"

using namespace kc1fsz;

const uint LED_PIN = 25;

static int int_pin_0 = 0;
static bool int_flag_0 = false;
static int int_pin_1 = 0;
static bool int_flag_1 = false;

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == int_pin_0)
        int_flag_0 = true;
    else if (gpio == int_pin_1)
        int_flag_1 = true;
}

int main(int, const char**) {

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
      
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);

    printf("LoRa Driver Demonstration 1\n");

    int reset_pin_0 = 1;
    int_pin_0 = 2;

    gpio_init(reset_pin_0);

    gpio_init(int_pin_0);
    gpio_set_irq_enabled_with_callback(int_pin_0, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    PicoClock clock;

    // Setup the poll timers
    PicoPollTimer radio_poll_0;
    radio_poll_0.setIntervalUs(50 * 1000);

    SX1276Driver radio_0(clock, reset_pin_0);
    radio_0.reset_radio();

    while (true) {        

        if (int_flag_0) {
            radio_0.event_int();
            int_flag_0 = false;
        }
        if (int_flag_1) {
            int_flag_1 = false;
        }
        radio_0.check_for_interrupts();

        if (radio_poll_0.poll()) {
            radio_0.event_tick();
        }
    }
}