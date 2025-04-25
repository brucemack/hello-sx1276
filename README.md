Project Setup
=============

Initialize submodules:

        git submodule update --init --recursive

Hardware Setup
==============

    spi_sck_pin_0 = 4;
    spi_mosi_pin_0 = 5;
    spi_miso_pin_0 = 6;
    spi_cs_pin_0 = 7;
    reset_pin_0 = 9;
    int_pin_0 = 10;

WARS Radio Board Hookup (J1)

* 1: 3.3V out
* 2: Radio interrupt out
* 3: 
* 4: Radio reset in
* 5: 
* 6: Radio SPI CS in
* 7: 5V in 
* 8: Radio SPI SCK in 
* 9: 
* 10: Radio SPI MOSI in
* 11: 
* 12: Radio SPI MISO out 
* 13: GND
* 14: GND
* 15: GND
* 16: GND

