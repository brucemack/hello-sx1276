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

    // ----- Radio 0 Hookup 

    int spi_sck_pin_0 = 2;
    int spi_mosi_pin_0 = 3;
    int spi_miso_pin_0 = 4;
    int spi_cs_pin_0 = 5;
    int reset_pin_0 = 6;
    int_pin_0 = 7;

    gpio_init(reset_pin_0);
    gpio_set_dir(reset_pin_0, GPIO_OUT);
    gpio_put(reset_pin_0, 1);

    gpio_init(int_pin_0);
    gpio_set_dir(int_pin_0, GPIO_IN);
    gpio_set_irq_enabled_with_callback(int_pin_0, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // The CS pin is not behaving the way we want, so drive manually
    gpio_init(spi_cs_pin_0);
    gpio_set_dir(spi_cs_pin_0, GPIO_OUT);
    gpio_put(spi_cs_pin_0, 1);

    gpio_set_function(spi_sck_pin_0, GPIO_FUNC_SPI);
    gpio_set_function(spi_mosi_pin_0, GPIO_FUNC_SPI);
    gpio_set_function(spi_miso_pin_0, GPIO_FUNC_SPI);
    spi_init(spi0, 500000);

    // ----- Radio 1 Hookup 

    int spi_sck_pin_1 = 10;
    int spi_mosi_pin_1 = 11;
    int spi_miso_pin_1 = 12;
    int spi_cs_pin_1 = 13;
    int reset_pin_1 = 14;
    int_pin_1 = 15;

    gpio_init(reset_pin_1);
    gpio_set_dir(reset_pin_1, GPIO_OUT);
    gpio_put(reset_pin_1, 1);

    gpio_init(int_pin_1);
    gpio_set_dir(int_pin_1, GPIO_IN);
    // First irq installs callback for all
    gpio_set_irq_enabled(int_pin_1, GPIO_IRQ_EDGE_RISE, true);

    // The CS pin is not behaving the way we want, so drive manually
    gpio_init(spi_cs_pin_1);
    gpio_set_dir(spi_cs_pin_1, GPIO_OUT);
    gpio_put(spi_cs_pin_1, 1);

    gpio_set_function(spi_sck_pin_1, GPIO_FUNC_SPI);
    gpio_set_function(spi_mosi_pin_1, GPIO_FUNC_SPI);
    gpio_set_function(spi_miso_pin_1, GPIO_FUNC_SPI);
    spi_init(spi1, 500000);

    PicoClock clock;
    PicoPollTimer radio_poll;
    radio_poll.setIntervalUs(50 * 1000);
    Log log;

    log.info("LoRa Driver Demonstration 1");

    SX1276Driver radio_0(log, clock, reset_pin_0, spi_cs_pin_0, spi0);
    SX1276Driver radio_1(log, clock, reset_pin_1, spi_cs_pin_1, spi1);

    // Start things by sending a message
    radio_0.send((const uint8_t*)"HELLO IZZY!", 10);

    while (true) {        

        // Look for interrupt activity and notify the radios
        uint32_t int_state = save_and_disable_interrupts();
        if (int_flag_0) {
            radio_0.event_int();
            int_flag_0 = false;
        }
        if (int_flag_1) {
            radio_1.event_int();
            int_flag_1 = false;
        }        
        restore_interrupts(int_state);

        // Fast poll
        radio_0.event_poll();
        radio_1.event_poll();

        // Slow poll
        if (radio_poll.poll()) {
            radio_0.event_tick();
            radio_1.event_tick();
        }

        // Check for receive activity on both radios
        uint8_t buf[256];
        unsigned int buf_len = 256;
        if (radio_0.popReceiveIfNotEmpty(0, buf, &buf_len)) {
            log.info("Radio 0 got %d", buf_len);
            prettyHexDump(buf, buf_len, std::cout);
        }
        if (radio_1.popReceiveIfNotEmpty(0, buf, &buf_len)) {
            log.info("Radio 1 got %d", buf_len);
            prettyHexDump(buf, buf_len, std::cout);
            // Echo
            radio_1.send(buf, buf_len);
        }
    }
}
