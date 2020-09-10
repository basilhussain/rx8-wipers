/*
uart.c

Copyright (c) 2020, Basil Hussain
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "uart.h"

// #TODO: interrupt-driven implementation doesn't work after power-down sleep.
// manages to transmit two characters ("Wo" of "Woke up!") then hangs.

#ifdef UART_IMPL_INTERRUPT
static uart_buffer_status_t uart_buffer_write(volatile uart_buffer_t * buffer, uint8_t byte);
static uart_buffer_status_t uart_buffer_read(volatile uart_buffer_t * buffer, uint8_t * byte);
static bool uart_buffer_is_empty(volatile uart_buffer_t * buffer);

static volatile uart_buffer_t uart_tx_buffer = { { 0 }, 0, 0 };
#endif /* UART_IMPL_INTERRUPT */

void uart_init(void) {
	// Set pin PA1 as output for TX and PA2 as input for RX (not that we are
	// using it!).
	DDRA |= _BV(DDA1);
	DDRA &= ~(_BV(DDA2));

	// Enable internal pull-up resistor on RX pin.
	PUEA |= _BV(PUEA2);

	// Put TX line in high idle state.
	PORTA |= _BV(PORTA1);

	// Set baud rate register according to value auto-calculated from BAUD
	// define in setbaud.h.
	UBRR0 = UBRR_VALUE;

	// Also set bit in according to whether it thinks we need 2X mode for the
	// specified baud rate.
#if USE_2X
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	// Set to 8-bit data, no parity, 1 stop bit, then enable TX only.
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	UCSR0B = _BV(TXEN0);
}

int uart_putchar(char c, FILE * stream) {
	// If character to transmit is LF, send a CR preceding it.
    if(c == '\n') uart_putchar('\r', stream);

#ifdef UART_IMPL_INTERRUPT
	// Write the character into the TX buffer. If it's full, keep trying until
	// it's not.
	while(uart_buffer_write(&uart_tx_buffer, c) == UART_BUFFER_FULL);

	// Enable the data-register-empty interrupt so that transmission of the
	// buffer can begin in the background.
	UCSR0B |= _BV(UDRIE0);
#endif /* UART_IMPL_INTERRUPT */

#ifdef UART_IMPL_BLOCKING
	// Wait until the data register is empty, clear any transmit complete flag,
	// then finally write the byte to be transmitted to the data register.
    loop_until_bit_is_set(UCSR0A, UDRE0);
	UCSR0A |= _BV(TXC0);
    UDR0 = c;
#endif /* UART_IMPL_BLOCKING */

	return 0;
}

void uart_flush(void) {

#ifdef UART_IMPL_INTERRUPT
	// Wait until both TX buffer is empty and any in-progress transmission is
	// flagged as complete.
	while(!uart_buffer_is_empty(&uart_tx_buffer) || bit_is_clear(UCSR0A, TXC0));
#endif /* UART_IMPL_INTERRUPT */

#ifdef UART_IMPL_BLOCKING
	// Wait until the transmission complete flag is set, signaling that the
	// transmit shift register is now empty.
	loop_until_bit_is_set(UCSR0A, TXC0);
#endif /* UART_IMPL_BLOCKING */

}

#ifdef UART_IMPL_INTERRUPT

ISR(USART0_UDRE_vect) {
	uint8_t data_tmp = 0;

	// Attempt to pull the next byte to be transmitted from the TX buffer. If
	// unsuccessful because buffer was empty, disable this ISR as there's
	// nothing more to do.
	if(uart_buffer_read(&uart_tx_buffer, &data_tmp) == UART_BUFFER_EMPTY) {
		UCSR0B &= ~(_BV(UDRIE0));
	} else {
		// Clear any transmit complete flag, then write the byte to be
		// transmitted to the data register.
		UCSR0A |= _BV(TXC0);
		UDR0 = data_tmp;
	}
}

uart_buffer_status_t uart_buffer_write(volatile uart_buffer_t * buffer, uint8_t byte) {
	// Calculate head index in buffer for new data byte to be written. Wrap if
	// necessary.
	uint8_t head_new = (buffer->head + 1) & UART_BUFFER_SIZE_MASK;

	// Check there is free space in buffer.
	if(head_new != buffer->tail) {
		// Store new data byte in buffer.
		buffer->data[head_new] = byte;

		// Store new head index.
		buffer->head = head_new;

		return UART_BUFFER_OK;
	} else {
		return UART_BUFFER_FULL;
	}
}

uart_buffer_status_t uart_buffer_read(volatile uart_buffer_t * buffer, uint8_t * byte) {
	// Check if buffer is empty.
	if(buffer->head != buffer->tail) {
		// Calculate new buffer tail index of next data byte to be read. Wrap
		// around if necessary.
		buffer->tail = (buffer->tail + 1) & UART_BUFFER_SIZE_MASK;

		// Copy data byte to output argument.
		*byte = buffer->data[buffer->tail];

		return UART_BUFFER_OK;
	} else {
		return UART_BUFFER_EMPTY;
	}
}

bool uart_buffer_is_empty(volatile uart_buffer_t * buffer) {
	return (buffer->head == buffer->tail);
}

#endif /* UART_IMPL_INTERRUPT */
