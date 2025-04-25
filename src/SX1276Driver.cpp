#include "SX1276Driver.h"

namespace kc1fsz {

SX1276Driver::SX1276Driver() : 
    _txBuffer(0),
    _rxBuffer(2) {
}

void SX1276Driver::init() {

}

/**
 * @brief This should be called by the ISR
 */
void SX1276Driver::isr() {
    // NOTE: We've had so many problems with interrupt enable/disable on the ESP32
    // so now we're just going to set a flag and let everything happen in the loop()
    // context. This eliminates a lot of risk around concurrency, etc.
    _isrHit = true;
}

void SX1276Driver::start_Tx() {

    //logger.println("start_Tx");

    // At this point we have something pending to be sent.
    // Go into stand-by so we are allowed to fill the FIFO
    // (see page 34)
    set_mode_STDBY();

    // Pop the data off the TX queue into the transmit buffer.  
    unsigned int tx_buf_len = 256;
    uint8_t tx_buf[tx_buf_len];
    txBuffer.pop(0, tx_buf, &tx_buf_len);

    // Move the data into the radio FIFO
    write_message(tx_buf, tx_buf_len);
    
    // Go into transmit mode
    state = State::TX_STATE;
    startTxTime = mainClock.time();
    enable_interrupt_TxDone();
    set_mode_TX();
}

/**
 * @brief Put the radio into receive mode and enable the RxDone interrupt.
 */
void SX1276Driver::start_Rx() {
    state = State::RX_STATE;
    startRxTime = mainClock.time();
    // Ask for interrupt when receiving
    enable_interrupt_RxDone();
    set_mode_RXCONTINUOUS();
}

/**
 * @brief Put the radio into CAD (channel activity detect) mode
 * and enable the CadDone interrupt.
 * 
 * NOTE: It appears that we don't get the CadDone interrupt under
 * normal (inactive) circumstances.  So a timeout is used to stop
 * the CAD process later.
 */
void SX1276Driver::start_Cad() {      

    //logger.println("start_Cad");

    state = State::CAD_STATE;
    startCadTime = mainClock.time();
    enable_interrupt_CadDone();
    set_mode_CAD();  
}

void SX1276Driver::start_Idle() {
    state = State::IDLE_STATE;
    set_mode_STDBY();
}

/**
 * @brief This is called when the radio reports the end of a 
 * transmission sequence.
 * 
 * This gets called from the main processing loop.
 */
void SX1276Driver::event_TxDone(uint8_t irqFlags) {   

    //logger.println("TxDone");

    // If we expected this event
    if (state == State::TX_STATE) {
        // After transmit the radio goes back to standby mode
        // automatically
        state = State::IDLE_STATE;
    }
    // Unexpected event
    else {
        logger.println("WRN: Unexpected TxDone");
        start_Idle();
    }
} 

/**
 * @brief This is called when a complete message is received.
 * 
 * This gets called from the main processing loop.
 */
void SX1276Driver::event_RxDone(uint8_t irqFlags) {

    //logger.println("RxDone");

    // Record that some activity was seen on the channel
    lastActivityTime = mainClock.time();

    // Expected
    if (state == State::RX_STATE) {
        // Make sure we don't have any errors
        if (irqFlags & 0x20) {
            if (systemConfig.getLogLevel() > 0) {
               logger.println("WRN: CRC error");
            }
            // Message is ignored
        }
        else {
            // How much data is available? RxBytesNb
            const uint8_t len = spi_read(0x13);

            // We do nothing for zero-length messages
            if (len == 0) {
                return;
            }

            // Set the FIFO read pointer to the beginning of the 
            // packet we just got. FifoAddrPtr=FifoRxCurrentAddr.  
            spi_write(0x0d, spi_read(0x10));

            // Stream received data in from the FIFO. 
            uint8_t rx_buf[256];
            spi_read_multi(0x00, rx_buf, len);
            
            // Grab the RSSI value from the radio
            int8_t lastSnr = (int8_t)spi_read(0x19) / 4;
            int16_t lastRssi = spi_read(0x1a);
            if (lastSnr < 0)
                lastRssi = lastRssi + lastSnr;
            else
                lastRssi = (int)lastRssi * 16 / 15;
            // We are using the high frequency port
            lastRssi -= 157;

            // Put the RSSI (OOB) and the entire packet into the circular queue for 
            // later processing.
            rxBuffer.push((const uint8_t*)&lastRssi, rx_buf, len);
        }
        // NOTE: Stay in RX state
    }
    // Unexpected 
    else {
        logger.println("WRN: Unexpected RxDone");
        start_Idle();
    }
}

/**
 * @brief This function is called when the radio completes a CAD cycle
 * and it is determined that these is channel activity. 
 */
void SX1276Driver::event_CadDone(uint8_t irqFlags) {

    // Expected
    if (state == State::CAD_STATE) {
        // This is the case where activity was detected
        if (irqFlags & 0x01) {
            logger.println("INF: CadDone Detection");
            lastActivityTime = mainClock.time();
            // Radio goes back to standby automatically after CAD
            state = State::IDLE_STATE;
        }
    }
    // Unexpected
    else {
        logger.println("WRN: Unexpected CadDone");
        start_Idle();
    }
}

/**
 * @brief This function gets called from inside of the main processing loop.  It looks
 * to see if any interrupt activity has been detected and, if so, figure 
 * out what kind of interrupt was reported and calls the correct handler.
 */ 
void SX1276Driver::check_for_interrupts() {

    // Look at the flag that gets set by the ISR itself
    if (!isrHit) {
        return;
    } else {
        if (systemConfig.getLogLevel() > 0) {
            logger.println("INF: Int");
        }
    }

    // *******************************************************************************
    // Critical Section:
    // Here we make sure that the clearing of the isr_hit flag and the unloading 
    // of the pending interrupts in the radio's IRQ register happen atomically.
    // We are avoding the case where 
    noInterrupts();

    isrHit = false;

    // Read and reset the IRQ register at the same time:
    uint8_t irq_flags = spi_write(0x12, 0xff);    
    
    // We saw a comment in another implementation that said that there are problems
    // clearing the ISR sometimes.  Notice we do a logical OR so we don't loose anything.
    irq_flags |= spi_write(0x12, 0xff);    

    interrupts();
    // *******************************************************************************
    
    // RxDone 
    if (irq_flags & 0x40) {
        event_RxDone(irq_flags);
    }
    // TxDone
    if (irq_flags & 0x08) {
        event_TxDone(irq_flags);
    }
    // CadDone
    if (irq_flags & 0x04) {
        event_CadDone(irq_flags);
    }
}

void SX1276Driver::event_tick_Idle() {

    // Make sure the radio is in the state that we expect
    uint8_t state = spi_read(0x01);  
    if (state != 0x81) {
        logger.println("WRN: Radio in unexpected state.");
        reset_radio();
        return;
    }
      
    // Check to see if there is data waiting to go out
    if (!txBuffer.isEmpty()) {
        // Launch a CAD check to see if the channel is clear
        start_Cad(); 
    } 
    else {
        start_Rx();
    }
}

void SX1276Driver::event_tick_Tx() {
    // Check for the case where a transmission times out
    if (mainClock.time() - startTxTime > TX_TIMEOUT_MS) {
        logger.println("ERR: TX time out");
        start_Idle();
    }
}

void SX1276Driver::event_tick_Rx() {

    // Check for pending transmissions.  If nothing is pending then 
    // return without any state change.
    if (!txBuffer.isEmpty()) {
        // At this point we know there is something pending.  We first 
        // go into CAD mode to make sure the channel is innactive.
        // A successful CAD check (with no detection) will trigger 
        // the transmission.
        start_Cad();
    }

    // Check for the case where a receive times out
    if (mainClock.time() - startRxTime > RX_TIMEOUT_MS) {
        start_Idle();
    }
}

void SX1276Driver::event_tick_Cad() {
    // Check for the case where a CAD check times out.  A random timeout is 
    // used to try to reduce collisions
    if ((mainClock.time() - startCadTime) > (CAD_TIMEOUT_MS * random(1, 3))) {
        // If a CAD times out then that means that it is safe 
        // to transmit. 
        if (!txBuffer.isEmpty()) {
            start_Tx();
        } else {
            start_Idle();
        }
    }
}

// Call periodically to look for timeouts or other pending activity.  
// This will happen on the regular application thread.
bool SX1276Driver::event_tick(void*) {
    if (state == State::RX_STATE) {
        event_tick_Rx();
    } else if (state == State::TX_STATE) {
        event_tick_Tx();
    } else if (state == State::CAD_STATE) {
        event_tick_Cad();
    } else if (state == State::IDLE_STATE) {
        event_tick_Idle();
    } 
    return true;
}

// --------------------------------------------------------------------------
// Radio Utilty Functions
// 
// This reference will be very important to you:
// https://www.hoperf.com/data/upload/portal/20190730/RFM95W-V2.0.pdf
// 
// The LoRa register map starts on page 103.

void SX1276Driver::set_mode_SLEEP() {
    spi_write(0x01, 0x00);  
}

void SX1276Driver::set_mode_STDBY() {
    spi_write(0x01, 0x01);  
}

void SX1276Driver::set_mode_TX() {
    spi_write(0x01, 0x03);
}

void SX1276Driver::set_mode_RXCONTINUOUS() {
    spi_write(0x01, 0x05);
}

void SX1276Driver::set_mode_RXSINGLE() {
    spi_write(0x01, 0x06);
}

void SX1276Driver::set_mode_CAD() {
    spi_write(0x01, 0x07);
}

// See table 17 - DIO0 is controlled by bits 7-6
void SX1276Driver::enable_interrupt_TxDone() {
    spi_write(0x40, 0x40);
}

// See table 17 - DIO0 is controlled by bits 7-6
void SX1276Driver::enable_interrupt_RxDone() {
    spi_write(0x40, 0x00);
}

// See table 17 - DIO0 is controlled by bits 7-6
void SX1276Driver::enable_interrupt_CadDone() {
    spi_write(0x40, 0x80);
}

/** Sets the radio frequency from a decimal value that is quoted
 *   in MHz.
 */
// See page 103
void SX1276Driver::set_frequency(float freq_mhz) {
    const float CRYSTAL_MHZ = 32000000.0;
    const float FREQ_STEP = (CRYSTAL_MHZ / 524288);
    const uint32_t f = (freq_mhz * 1000000.0) / FREQ_STEP;
    spi_write(0x06, (f >> 16) & 0xff);
    spi_write(0x07, (f >> 8) & 0xff);
    spi_write(0x08, f & 0xff);
}

void SX1276Driver::write_message(uint8_t* data, uint8_t len) {
    // Move pointer to the start of the FIFO
    spi_write(0x0d, 0);
    // The message
    spi_write_multi(0x00, data, len);
    // Update the length register
    spi_write(0x22, len);
}

int SX1276Driver::reset_radio() {
  
    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, HIGH);
    delay(5);
    digitalWrite(RST_PIN, LOW);
    delay(5);
    digitalWrite(RST_PIN, HIGH);
    // Float the reset pin
    pinMode(RST_PIN, INPUT);
    // Per datasheet, wait 5ms after reset
    delay(5);
    // Not sure if this is really needed:
    delay(250);

    // Initialize the radio
    if (init_radio() != 0) {
        logger.println(F("ERR: Problem with radio initialization"));
        return -1;
    }

    logger.println(F("INF: Radio initialized"));

    // Flash the LED as a diagnostic indicator 
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    start_Idle();

    return 0;  
}

/**
 * @brief Sets the Over Current Protection register.
 * 
 * @param current_ma 
 */
void SX1276Driver::set_ocp(uint8_t current_ma) {

    uint8_t trim = 27;
    if (current_ma <= 120) {
        trim = (current_ma - 45) / 5;
    } else if (current_ma <= 240) {
        trim = (current_ma + 30) / 10;
    }
    spi_write(0x0b, 0x20 | (0x1F & trim));
}

void SX1276Driver::setLowDatarate() {

    // called after changing bandwidth and/or spreading factor
    //  Semtech modem design guide AN1200.13 says 
    // "To avoid issues surrounding  drift  of  the  crystal  reference  oscillator  due  to  either  temperature  change  
    // or  motion,the  low  data  rate optimization  bit  is  used. Specifically for 125  kHz  bandwidth  and  SF  =  11  and  12,  
    // this  adds  a  small  overhead  to increase robustness to reference frequency variations over the timescale of the LoRa packet."
 
    // read current value for BW and SF
    uint8_t bw = spi_read(0x1d) >> 4;	// bw is in bits 7..4
    uint8_t sf = spi_read(0x1e) >> 4;	// sf is in bits 7..4
   
    // calculate symbol time (see Semtech AN1200.22 section 4)
    float bw_tab[] = { 7800, 10400, 15600, 20800, 31250, 41700, 62500, 
      125000, 250000, 500000};
    float bandwidth = bw_tab[bw];
    float symbolTime = 1000.0 * pow(2, sf) / bandwidth;	// ms
   
    // the symbolTime for SF 11 BW 125 is 16.384ms. 
    // and, according to this :- 
    // https://www.thethingsnetwork.org/forum/t/a-point-to-note-lora-low-data-rate-optimisation-flag/12007
    // the LDR bit should be set if the Symbol Time is > 16ms
    // So the threshold used here is 16.0ms
 
    // the LDR is bit 3 of register 0x26
    uint8_t current = spi_read(0x26) & ~0x08; // mask off the LDR bit
    if (symbolTime > 16.0)
      spi_write(0x26, current | 0x08);
    else
      spi_write(0x26, current);   
}

/** 
 *  All of the one-time initialization of the radio
 */
int SX1276Driver::init_radio() {

    // Check the radio version to make sure things are connected
    uint8_t ver = spi_read(0x42);
    if (ver != 18) {
        return -1;
    }

    // Switch into Sleep mode, LoRa mode
    spi_write(0x01, 0x80);
    // Wait for sleep mode 
    delay(10); 

    // Make sure we are actually in sleep mode
    if (spi_read(0x01) != 0x80) {
        return -1; 
    }

    // Setup the FIFO pointers
    // TX base:
    spi_write(0x0e, 0);
    // RX base:
    spi_write(0x0f, 0);

    set_frequency(STATION_FREQUENCY);

    // Set LNA boost
    spi_write(0x0c, spi_read(0x0c) | 0x03);

    // AgcAutoOn=LNA gain set by AGC
    spi_write(0x26, 0x04);

    // DAC enable (adds 3dB)
    spi_write(0x4d, 0x87);

    // Turn on PA and set power to +20dB
    // PaSelect=1
    // OutputPower=17 (20 - 3dB from DAC)
    spi_write(0x09, 0x80 | ((20 - 3) - 2));

    // Set OCP to 140 (as per the Sandeep Mistry library)
    set_ocp(140);

    // Go into stand-by
    set_mode_STDBY();

    // Configure the radio
    uint8_t reg = 0;

    // 7-4: 0111  (125k BW)
    // 3-1: 001   (4/5 coding rate)
    // 0:   0     (Explicit header mode)
    reg = 0b01110010;
    spi_write(0x1d, reg);

    // 7-4:   9 (512 chips/symbol, spreading factor 9)
    // 3:     0 (TX continuous mode normal)
    // 2:     1 (CRC mode on)
    // 1-0:   0 (RX timeout MSB) 
    reg = 0b10010100;
    spi_write(0x1e, reg);

    // Preable Length=8 (default)
    // Preamble MSB and LSB
    //spi_write(0x20, 8 >> 8);
    //spi_write(0x21, 8 & 0xff);

    setLowDatarate();

    return 0;
}

}
