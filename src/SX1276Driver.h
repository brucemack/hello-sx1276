#ifndef _SX1276Driver_h
#define _SX1276Driver_h

#include <stdint.h>
#include "hardware/spi.h"

#include <kc1fsz-tools/Log.h>
#include <kc1fsz-tools/CircularBuffer.h>
#include <kc1fsz-tools/Clock.h>

namespace kc1fsz {

class SX1276Driver {

public:

    SX1276Driver(Log& log, Clock& clock, int reset_pin, spi_inst_t* spi);

    int reset_radio();
    // Called by interrupt pin
    void event_int();
    // This should be called every 50-100ms.
    void event_tick();
    // This should be called as quickly/often as possible
    void event_poll();

private:

    void start_Tx();
    void start_Rx();
    void start_Cad();
    void start_Idle();
    void event_TxDone(uint8_t irqFlags);
    void event_RxDone(uint8_t irqFlags);
    void event_CadDone(uint8_t irqFlags);
    void event_tick_Idle();
    void event_tick_Tx();
    void event_tick_Rx();
    void event_tick_Cad();

    void set_mode_SLEEP();
    void set_mode_STDBY();
    void set_mode_TX();    
    void set_mode_RXCONTINUOUS();
    void set_mode_RXSINGLE();
    void set_mode_CAD();
    
    void enable_interrupt_TxDone();    
    void enable_interrupt_RxDone();
    void enable_interrupt_CadDone();
    void set_frequency(float freq_mhz);
    void write_message(uint8_t* data, uint8_t len);
    void set_ocp(uint8_t current_ma);
    void set_low_datarate();
    int init_radio(); 

    uint8_t spi_read(uint8_t reg);

    /**
     * @param len must be <= 255 bytes!
     */
    void spi_read_multi(uint8_t reg, uint8_t* buf, uint8_t len);

    /**
     * @returns Whatever comes back during the second byte transfer.
     */
    uint8_t spi_write(uint8_t reg, uint8_t val);

    /**
     * @param len must be <= 255 bytes!
     * @returns Whatever comes back during the second byte transfer.
     */
    uint8_t spi_write_multi(uint8_t reg, const uint8_t* buf, uint8_t len);

    uint32_t _random(uint32_t from, uint32_t to);

    /**
     * Causes a sleep
     * @param sleep_ms The delay time in milliseconds
     */
    void _delay(int ms);

    Log& logger;
    Clock& _mainClock;
    int _resetPin;

    // The states of the state machine
    enum State { IDLE_STATE, RX_STATE, TX_STATE, CAD_STATE };

    // The overall state
    volatile State _state = State::IDLE_STATE;
    // This is volatile because it is set inside of the ISR context
    volatile bool _isrHit = false;

    // The time when the last receive was started.  Used to manage
    // timeouts.
    volatile uint32_t _startRxTime = 0;
    // The time when we started the last transmission.  This is needed 
    // to create a timeout on transmissions so we don't accidentally get 
    // stuck in a transmission.
    volatile uint32_t _startTxTime = 0;
    // The time when we started the last channel activity detection.
    // This is needed to manage timeouts.
    volatile uint32_t _startCadTime = 0;
    // The time when the last channel activity was seen.. Used to avoid
    // transmission when there is receive activity going on.
    volatile uint32_t _lastActivityTime = 0;

    // We keep a pretty small TX buffer because the main area where we keep 
    // outbound packets is in the MessageProcessor.
    CircularBufferImpl<256> _txBuffer;
    // There is a two-byte OOB allocation here for the RSSI data on receive
    CircularBufferImpl<2048> _rxBuffer;

    spi_inst_t* _spi;
};

}

#endif
