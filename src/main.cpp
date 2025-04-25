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
#include "hardware/spi.h"
#include "hardware/sync.h"

#include "kc1fsz-tools/Log.h"
#include "kc1fsz-tools/rp2040/PicoClock.h"
#include "kc1fsz-tools/rp2040/PicoPollTimer.h"
#include "SX1276Driver.h"

using namespace kc1fsz;

const uint LED_PIN = 25;

static int int_pin_0 = 0;
static int int_pin_1 = 0;
static volatile bool int_flag_0 = false;
static volatile bool int_flag_1 = false;

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

    int reset_pin_0 = 9;
    int_pin_0 = 10;

    int spi_sck_pin_0 = 4;
    int spi_mosi_pin_0 = 5;
    int spi_miso_pin_0 = 6;
    int spi_cs_pin_0 = 7;

    gpio_init(reset_pin_0);
    gpio_set_dir(reset_pin_0, GPIO_OUT);
    gpio_put(reset_pin_0, 1);

    gpio_init(int_pin_0);
    gpio_set_dir(reset_pin_0, GPIO_IN);
    gpio_set_irq_enabled_with_callback(int_pin_0, 
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    gpio_set_function(spi_cs_pin_0, GPIO_FUNC_SPI);
    gpio_set_function(spi_sck_pin_0, GPIO_FUNC_SPI);
    gpio_set_function(spi_mosi_pin_0, GPIO_FUNC_SPI);
    gpio_set_function(spi_miso_pin_0, GPIO_FUNC_SPI);
    spi_init(spi0, 500000);

    PicoClock clock;

    // Setup the poll timers
    PicoPollTimer radio_poll_0;
    radio_poll_0.setIntervalUs(50 * 1000);

    Log logger;

    SX1276Driver radio_0(logger, clock, reset_pin_0, spi0);
    radio_0.reset_radio();

    while (true) {        

        uint32_t int_state = save_and_disable_interrupts();
        if (int_flag_0) {
            radio_0.event_int();
            int_flag_0 = false;
        }
        if (int_flag_1) {
            int_flag_1 = false;
        }        
        restore_interrupts(int_state);

        radio_0.event_poll();

        if (radio_poll_0.poll()) {
            radio_0.event_tick();
        }
    }
}