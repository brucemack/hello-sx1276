Project Setup
=============

Initialize submodules:

        git submodule update --init --recursive

Build/Run
=========

        ~/git/openocd/src/openocd -s ~/git/openocd/tcl -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 500" -c "program main.elf verify reset exit"

Hardware Setup
==============

    spi_sck_pin_0 = 2;
    spi_mosi_pin_0 = 3;
    spi_miso_pin_0 = 4;
    spi_cs_pin_0 = 5;
    reset_pin_0 = 6;
    int_pin_0 = 7;

WARS Radio Board 2022-04 Hookup (J1)

* 1: 3.3V out
* 3: 
* 5: 
* 7: 5V in 
* 9: 
* 11: 
* 13: GND
* 15: GND

* 2: Radio interrupt out
* 4: Radio reset in
* 6: Radio SPI CS in
* 8: Radio SPI SCK in 
* 10: Radio SPI MOSI in
* 12: Radio SPI MISO out 
* 14: GND
* 16: GND

