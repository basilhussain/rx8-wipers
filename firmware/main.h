/*
main.h

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

#ifndef MAIN_H_
#define MAIN_H_

// ATtiny841 chips have an internal 8MHz oscillator calibrated for operation at
// 3.3V. Default value is a general adjustment necessary for operation at 5V.
// (Tested with one sample measured as operating at 8.338MHz @ 5V; reduced to
// 7.988MHz with adjustment of -8. Appears to work well on multiple samples.)
#define OSCCAL_EEPROM_ADDR 0
#define OSCCAL_EEPROM_DEFAULT -8

#define CONFIG_EEPROM_ADDR 1

#define INTVOL_SETTINGS_COUNT 9

#define WIPE_AUTOSTOP_HIGH_TIMEOUT_MS 1500
#define WIPE_AUTOSTOP_LOW_TIMEOUT_MS 4000

#define WASH_WIPE_TIMEOUT_EEPROM_DEFAULT 750 // standard behaviour is 2000
#define WASH_WIPE_COUNT_EEPROM_DEFAULT 3 // standard behaviour is 1

#define SPEED_MAP_VSS_THRESHOLD_COUNT 5

#define WAKE_UP_DELAY_MS 300

// Undefine this to disable speed-sensitive adjustment of wiper interval
// according to vehicle speed.
#define SPEED_MAP_ENABLED

// Undefine to disable going into standby when intermittent mode or wash are not
// active.
#define STANDBY_ENABLED

// Uncomment to enable code to emit ANSI control codes to clear screen and
// re-home cursor before printing status information every iteration of the main
// loop.
// #define STATUS_PRINT_CLS_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "uart.h"

// Macros to power-down and disable/enable all peripheral modules, or only
// modules that we are or are not using (don't need SPI, USART1 and TWI).
#define power_disable_all() (PRR |= _BV(PRTWI) | _BV(PRUSART1) | _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC))
#define power_enable_all() (PRR &= ~(_BV(PRTWI) | _BV(PRUSART1) | _BV(PRUSART0) | _BV(PRSPI) | _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC)))
#define power_disable_unused() (PRR |= _BV(PRTWI) | _BV(PRUSART1) | _BV(PRSPI))
#define power_enable_used() (PRR &= ~(_BV(PRUSART0) | _BV(PRTIM2) | _BV(PRTIM1) | _BV(PRTIM0) | _BV(PRADC)))

// Macros for turning wiper relay on/off.
#define relay_on() (PORTA |= _BV(PORTA7))
#define relay_off() (PORTA &= ~(_BV(PORTA7)))

typedef int32_t timestamp_t;
typedef struct {
	uint16_t int_vol_thresholds[INTVOL_SETTINGS_COUNT];
	uint16_t int_vol_intervals[INTVOL_SETTINGS_COUNT];
	uint16_t wash_wipe_timeout;
	uint8_t wash_wipe_count;
	uint16_t speed_map_vss_thresholds[SPEED_MAP_VSS_THRESHOLD_COUNT];
	uint8_t speed_map_int_vol_settings[INTVOL_SETTINGS_COUNT][SPEED_MAP_VSS_THRESHOLD_COUNT];
} config_t;

// All functions declared as static to reduce code size. Compiler will inline
// them if not called from more than one location.
static int8_t calibrate_oscillator();
static void init_io();
static void init_interrupts();
static void init_adc();
static void init_timers();
static void print_word_array(const uint16_t * array, const size_t size);
static bool validate_word_array_values_range(const uint16_t * array, const size_t size, const uint16_t min_val, const uint16_t max_val);
static bool validate_word_array_values_ascending(const uint16_t * array, const size_t size);
static void config_load_and_validate();
static void config_validate_int_vol_thresholds();
static void config_validate_int_vol_intervals();
static void config_validate_wash_wipe_timeout();
static void config_validate_wash_wipe_count();
static void config_validate_speed_map_vss_thresholds();
static void config_validate_speed_map_int_vol_settings();
static void check_vss();
static void update_switch_states();
static void update_int_vol_state();
static void wipe(const bool immediate);
static void wipe_actuate(uint8_t count);
static bool wipe_autostop_wait(const bool wait_for_state, const uint16_t timeout);
static void wash();
static void standby();

// #TODO: change int_sw @ wash_sw into struct as bit fields - will reduce memory from 2 bytes to 1, and allow expansion for further 6 vars without using more memory.
// #TODO: also change int_vol_* to struct? can pack vars, as most don't use all bits? e.g. adc_val = 10 bits, setting = 4 bits, interval = 16 bits = 30 bits => 32 bits/4 bytes - saves 1 byte!
// #TODO: maybe also vss_* vars?
// Although... will vars being in struct no longer guarantee atomic access for <= 8-bit vars?

FILE uart_output;
config_t config;
volatile bool int_sw = false;
volatile bool wash_sw = false;
bool int_sw_prev = false;
bool wash_sw_prev = false;
volatile uint16_t int_vol_adc_val = 0;
volatile uint8_t int_vol_setting_commanded = 0;
volatile uint8_t int_vol_setting_actual = 0;
volatile uint16_t int_vol_interval = 0;
volatile uint8_t vss_overflows = 0; // Make uint16_t if expected to measure freq > 65KHz
volatile uint16_t vss_frequency = 0;
volatile timestamp_t sys_timestamp = 0;
bool speed_map_disabled = false;

const uint16_t config_int_vol_thresholds_default[INTVOL_SETTINGS_COUNT] PROGMEM = {
	// Following upper threshold values are for switch with a maximum
	// resistance of 42.7K, ADC ref. voltage of 2.2V and resistor divider pair
	// value of 62K.
	62, 232, 402, 547, 667, 779, 878, 939, 1023
};
const uint16_t config_int_vol_intervals_default[INTVOL_SETTINGS_COUNT] PROGMEM = {
	// Interval timings in milliseconds.
	1800, 2200, 3500, 4600, 5600, 6800, 8000, 9100, 9700
	// 3600, 4000, 5300, 6400, 7400, 8600, 9800, 10900, 11500
};
const uint16_t config_speed_map_vss_thresholds_default[SPEED_MAP_VSS_THRESHOLD_COUNT] PROGMEM = {
	// VSS value threshold for each column in the table below.
	0, 5, 10, 15, 20
};
const uint8_t config_speed_map_int_vol_settings_default[INTVOL_SETTINGS_COUNT][SPEED_MAP_VSS_THRESHOLD_COUNT] PROGMEM = {
	// For each input intermittent mode setting, the setting that is actually
	// applied when speed is below (or equal to) each threshold above.
	// Row indices correspond to int. vol. setting values, 0-8 descending.
	{ 8, 5, 2, 1, 0 },
	{ 8, 6, 4, 3, 2 },
	{ 8, 6, 5, 4, 3 },
	{ 8, 7, 6, 5, 4 },
	{ 8, 7, 6, 5, 4 },
	{ 8, 8, 7, 6, 6 },
	{ 8, 8, 7, 7, 6 },
	{ 8, 8, 8, 7, 7 },
	{ 8, 8, 8, 8, 8 }
};

#endif /* MAIN_H_ */