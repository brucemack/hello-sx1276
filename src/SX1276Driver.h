#ifndef _SX1276Driver_h
#define _SX1276Driver_h

#include <kc1fsz-tools/CircularBuffer.h>

namespace kc1fsz {

class SX1276Driver {

public:

    SX1276Driver();

    void init();

private:

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
};

}

#endif
