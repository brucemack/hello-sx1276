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

#include "SX1276Driver.h"

using namespace kc1fsz;

const uint LED_PIN = 25;

int main(int, const char**) {

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
      
    gpio_put(LED_PIN, 1);
    sleep_ms(500);
    gpio_put(LED_PIN, 0);
    sleep_ms(500);

    printf("LoRa Driver Demonstration 1\n");

    while (true) {        
    }
}