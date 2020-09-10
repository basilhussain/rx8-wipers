/*
uart.h

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

#ifndef UART_H_
#define UART_H_

// NOTE: Do not forget to define BAUD, BAUD_TOL (optional), and one of
// UART_IMPL_BLOCKING or UART_IMPL_INTERRUPT to select the implementation to be
// used. When using the latter implementation, interrupts must be globally
// enabled either before or immediately after calling uart_init().

#define UART_BUFFER_SIZE 32
#define UART_BUFFER_SIZE_MASK (UART_BUFFER_SIZE - 1)
#if UART_BUFFER_SIZE > 256
#error "UART buffer size cannot be greater than 256"
#endif
#if (UART_BUFFER_SIZE & UART_BUFFER_SIZE_MASK)
#error "UART buffer size must be a power of 2 (e.g. 8, 16, 32, etc.)"
#endif

#if !defined(UART_IMPL_BLOCKING) && !defined(UART_IMPL_INTERRUPT)
#error "UART implementation choice not selected; define either UART_IMPL_BLOCKING or UART_IMPL_INTERRUPT"
#endif
#if defined(UART_IMPL_BLOCKING) && defined(UART_IMPL_INTERRUPT)
#error "Conflicting UART implementation choices defined; choose either UART_IMPL_BLOCKING or UART_IMPL_INTERRUPT"
#endif

#include <stdio.h>
#include <stdbool.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/setbaud.h>

#ifdef UART_IMPL_INTERRUPT
typedef enum {
	UART_BUFFER_OK, UART_BUFFER_EMPTY, UART_BUFFER_FULL
} uart_buffer_status_t;

typedef struct {
	uint8_t data[UART_BUFFER_SIZE];
	uint8_t head;
	uint8_t tail;
} uart_buffer_t;
#endif /* UART_IMPL_INTERRUPT */

void uart_init(void);
int uart_putchar(char c, FILE *stream);
void uart_flush(void);

#endif /* UART_H_ */