/*
 * serial_driver.h
 * Studio 13: Sensor Mini-Project
 *
 * Serial transport layer and packet framing.
 *
 * USE_BAREMETAL_SERIAL controls which transport path is compiled in:
 *
 *   0 (default) - sendFrame() uses Serial.write() and receiveFrame()
 *                 uses Serial.available() / Serial.read().  The sketch
 *                 works immediately with no further changes needed, so
 *                 you can test the E-Stop and color sensor activities
 *                 before touching the serial driver.
 *
 *   1           - sendFrame() and receiveFrame() use the circular
 *                 tx_buf / rx_buf buffers driven by the four stubs you
 *                 implement in Activity 1.  Change this to 1 once
 *                 txEnqueue(), rxDequeue(), and both ISRs are working.
 */

#pragma once

#include <avr/interrupt.h>
#include <string.h>
#include "packets.h"

// =============================================================
// TODO (Activity 1, step 2): change this to 1 once txEnqueue(),
// rxDequeue(), and both ISRs are implemented and working.
// =============================================================
#define USE_BAREMETAL_SERIAL 1

// =============================================================
// Circular TX / RX buffers (used when USE_BAREMETAL_SERIAL == 1)
// =============================================================

#if USE_BAREMETAL_SERIAL

// Power-of-2 sizes allow fast wrap-around with bitwise AND.
#define TX_BUFFER_SIZE  128
#define TX_BUFFER_MASK  (TX_BUFFER_SIZE - 1)
#define RX_BUFFER_SIZE  256
#define RX_BUFFER_MASK  (RX_BUFFER_SIZE - 1)

volatile uint8_t tx_buf[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0, tx_tail = 0;

volatile uint8_t rx_buf[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0, rx_tail = 0;

// =============================================================
// USART0 initialisation (used when USE_BAREMETAL_SERIAL == 1)
// =============================================================

// Configure USART0 for 8N1 at the given baud rate with TX, RX, and
// RX Complete interrupt enabled.  ubrr = (F_CPU / (16 * baud)) - 1.
// For 9600 baud at 16 MHz: ubrr = 103.
void usartInit(uint16_t ubrr) {
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr);
    UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);  // 8 data bits, 1 stop bit
}

// =============================================================
// Activity 1, step 1: implement these four stubs
// =============================================================

/*
 * TODO (Activity 1): Implement txEnqueue().
 *
 * Attempt to copy all `len` bytes from `data` into the circular TX
 * buffer.  If the buffer does not have enough free space, do NOT
 * enqueue anything and return false.  If successful, enable the UDRE
 * interrupt (UDRIE0) and return true.  Must NOT block.
 */
bool txEnqueue(const uint8_t *data, uint8_t len) {
    // TODO
    uint8_t used = (tx_head - tx_tail) & TX_BUFFER_MASK;
    if (len > (TX_BUFFER_MASK - used)){
    return false;
    }
    else{
      for (uint8_t i = 0; i < len; i++){
      tx_buf[tx_head] = data[i];
      tx_head = (tx_head + 1) & TX_BUFFER_MASK;
      }
      
      UCSR0B |= 0b00100000;
      return true;
  }
}

// TODO (Activity 1): Implement the TX Data Register Empty ISR.
// Vector: USART0_UDRE_vect
// Drain one byte from tx_buf into UDR0; when the buffer is empty,
// clear the UDRIE0 bit to stop the ISR from firing.

ISR(USART0_UDRE_vect) {
  if (tx_tail != tx_head){
    UDR0 = tx_buf[tx_tail];
    tx_tail = (tx_tail + 1) & TX_BUFFER_MASK;
  }

  if(tx_tail == tx_head){
    UCSR0B &= 0b11011111; 
  }
}


/*
 * TODO (Activity 1): Implement rxDequeue().
 *
 * Attempt to copy `len` bytes out of the circular RX buffer into
 * `data`.  If fewer than `len` bytes are available, do NOT copy
 * anything and return false.  Must NOT block.
 */
bool rxDequeue(uint8_t *data, uint8_t len) {
    // TODO
  if (((rx_head - rx_tail) & RX_BUFFER_MASK) < len){
    return false;
  }
  else{
    for (uint8_t i = 0; i < len; i++){
      data[i] = rx_buf[rx_tail];
      rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;
    }
    return true;
  }
}

#endif

// TODO (Activity 1): Implement the RX Complete ISR.
// Vector: USART0_RX_vect
// Read UDR0 immediately.  If the buffer is not full, store the byte
// and advance the write index; otherwise discard it.

ISR(USART0_RX_vect) {
  // YOUR CODE HERE
  uint8_t data = UDR0;
  uint8_t next = (rx_head + 1) & RX_BUFFER_MASK;
  if (next != rx_tail){
    rx_buf[rx_head] = data;
    rx_head = next;
  }
}

// =============================================================
// Framing: magic number + XOR checksum (pre-implemented)
// =============================================================

static uint8_t computeChecksum(const uint8_t *data, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs ^= data[i];
    return cs;
}

/*
 * Wrap a TPacket in the 103-byte frame and send it.
 *
 * When USE_BAREMETAL_SERIAL == 0: writes directly via Serial.write().
 * When USE_BAREMETAL_SERIAL == 1: enqueues into the circular TX buffer
 * (busy-waits if the buffer is temporarily full).
 */
static void sendFrame(const TPacket *pkt) {
    uint8_t frame[FRAME_SIZE];
    frame[0] = MAGIC_HI;
    frame[1] = MAGIC_LO;
    memcpy(&frame[2], pkt, TPACKET_SIZE);
    frame[2 + TPACKET_SIZE] = computeChecksum((const uint8_t *)pkt, TPACKET_SIZE);
#if USE_BAREMETAL_SERIAL
    while (!txEnqueue(frame, FRAME_SIZE))
        ;   // wait for TX buffer space
#else
    Serial.write(frame, FRAME_SIZE);
#endif
}

/*
 * Try to extract one valid framed packet from incoming bytes.
 * Returns true when a valid packet is stored in *pkt; returns false if
 * there are not enough bytes yet or if a frame fails the checksum.
 *
 * When USE_BAREMETAL_SERIAL == 0: reads from the Arduino Serial buffer.
 * When USE_BAREMETAL_SERIAL == 1: reads from the circular rx_buf buffer.
 */
static bool receiveFrame(TPacket *pkt) {
#if USE_BAREMETAL_SERIAL
    while (((rx_head - rx_tail) & RX_BUFFER_MASK) >= FRAME_SIZE) {
        uint8_t hi = rx_buf[rx_tail];
        uint8_t lo = rx_buf[(rx_tail + 1) & RX_BUFFER_MASK];

        if (hi == MAGIC_HI && lo == MAGIC_LO) {
            uint8_t frame[FRAME_SIZE];
            for (uint8_t i = 0; i < FRAME_SIZE; i++)
                frame[i] = rx_buf[(rx_tail + i) & RX_BUFFER_MASK];

            uint8_t expected = computeChecksum(&frame[2], TPACKET_SIZE);
            if (frame[FRAME_SIZE - 1] == expected) {
                memcpy(pkt, &frame[2], TPACKET_SIZE);
                rx_tail = (rx_tail + FRAME_SIZE) & RX_BUFFER_MASK;
                return true;
            }
        }

        rx_tail = (rx_tail + 1) & RX_BUFFER_MASK;
    }
    return false;
#else
    static uint8_t state = 0;
    static uint8_t raw[TPACKET_SIZE];
    static uint8_t index = 0;

    while (Serial.available() > 0) {
        uint8_t byte = (uint8_t)Serial.read();

        switch (state) {
            case 0: // waiting for MAGIC_HI
                if (byte == MAGIC_HI) {
                    state = 1;
                }
                break;

            case 1: // waiting for MAGIC_LO
                if (byte == MAGIC_LO) {
                    state = 2;
                    index = 0;
                } else if (byte != MAGIC_HI) {
                    state = 0;
                }
                // else stay in state 1 if we got another MAGIC_HI
                break;

            case 2: // reading TPacket payload
                raw[index++] = byte;
                if (index >= TPACKET_SIZE) {
                    state = 3;
                }
                break;

            case 3: { // reading checksum
                uint8_t expected = computeChecksum(raw, TPACKET_SIZE);
                if (byte == expected) {
                    memcpy(pkt, raw, TPACKET_SIZE);
                    state = 0;
                    return true;
                }
                // checksum failed; resync
                state = (byte == MAGIC_HI) ? 1 : 0;
                break;
            }
        }
    }
    return false;
#endif
}
