/*
main.c

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

#include "main.h"

int main(void) {
	int8_t osccal_adjustment = calibrate_oscillator();

	power_disable_unused();

	init_io();
	init_interrupts();
	init_adc();
	init_timers();

	uart_init();
	fdev_setup_stream(&uart_output, uart_putchar, NULL, _FDEV_SETUP_WRITE);
	stdout = &uart_output;

	printf_P(PSTR("----------------------------------------\n"));
	printf_P(PSTR("OSCCAL0 = 0x%X (adjustment of %d)\n"), OSCCAL0, osccal_adjustment);

	config_load_and_validate();

	check_vss();

	// Make an initial read of the int and wash switches, as they may already be
	// set at start-up, but no interrupt yet to tell us that. Also update
	// intermittent mode volume state.
	update_switch_states();
	update_int_vol_state();

	// Enable global interrupts.
	// Note: move up to before first printf_P() once buffered interrupt UART implementation working!
	sei();

	// Some commonly-used strings in status output below. Only define once to
	// save memory.
	PGM_P on_str = PSTR("ON");
	PGM_P off_str = PSTR("OFF");

	while(1) {
		// Output some status information.
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#ifdef STATUS_PRINT_CLS_ENABLED
			// Emit ANSI control codes to clear screen and re-home cursor.
			printf_P(PSTR("\x1B[2J\x1B[H"));
#endif
			printf_P(PSTR("SYS TS = %ld\n"), sys_timestamp);
			printf_P(PSTR("INT SW = %S (prev = %S)\n"), (int_sw ? on_str : off_str), (int_sw_prev ? on_str : off_str));
			printf_P(PSTR("WASH SW = %S (prev = %S)\n"), (wash_sw ? on_str : off_str), (wash_sw_prev ? on_str : off_str));
			printf_P(PSTR("INTVOL ADC = %u; setting (cmd / act) = %u / %u; interval = %u ms\n"), int_vol_adc_val, int_vol_setting_commanded, int_vol_setting_actual, int_vol_interval);
			printf_P((speed_map_disabled ? PSTR("VSS = N/A (speed-mapping disabled)\n") : PSTR("VSS = %u Hz\n")), vss_frequency);
		}

		if(int_sw || wash_sw) {
			// Wipe only if wash switch is not active. Make it an immediate wipe
			// if the int. switch was previously off, otherwise wait for
			// interval as normal.
			if(int_sw && !wash_sw) wipe(!int_sw_prev);

			// Only do wash auto-wipe if wash switch was previously off; this is
			// so we only do one auto-wipe operation per switch activation.
			if(wash_sw && !wash_sw_prev) wash();

			// Update the previous switch states with current values.
			int_sw_prev = int_sw;
			wash_sw_prev = wash_sw;
		} else {
#ifdef STANDBY_ENABLED
			printf_P(PSTR("Going to sleep...\n"));
			standby();
			printf_P(PSTR("Woke up!\n"));

			// Small delay before continuing; this avoids triggering a wipe if
			// user is simply transitioning wiper stalk through the intermittent
			// position (e.g. between low-speed and off).
			_delay_ms(WAKE_UP_DELAY_MS);
#endif

			// Reset these both as they must have been off if we were asleep.
			int_sw_prev = false;
			wash_sw_prev = false;
		}

#ifndef STANDBY_ENABLED
		_delay_ms(200);
#endif
	}
}

int8_t calibrate_oscillator() {
	// Read the signed integer adjustment value from EEPROM.
	eeprom_busy_wait();
	int8_t osccal_adjust = (int8_t)eeprom_read_byte((const uint8_t *)OSCCAL_EEPROM_ADDR);

	// Sanity check the adjustment value. Firstly, check for empty EEPROM (0xFF)
	// or zero value. For empty check, because we are dealing with a signed
	// integer, it would otherwise be impossible to tell the difference between
	// empty and -1 (0xFF); so, if we mask the sign bit and then check for 0x7F
	// (i.e. all one bits), that should suffice. Finally, make sure that the
	// adjustment is not more than 0x20 (30), as that is the maximum step change
	// that can be made.
	if((osccal_adjust & 0x7F) == 0x7F || osccal_adjust == 0 || abs(osccal_adjust) > 0x20) {
		osccal_adjust = OSCCAL_EEPROM_DEFAULT;
	}

	// Make the adjustment!
	OSCCAL0 += osccal_adjust;

	return osccal_adjust;
}

void init_io() {
	// Set pin PA7 as an output. All others are inputs by default.
	DDRA |= _BV(DDA7);

	// Disable digital inputs on pins PA4-6, as these are normally unconnected
	// and floating (only used by ISP). May save a tiny bit of power?
	DIDR0 |= _BV(ADC4D) | _BV(ADC5D) | _BV(ADC6D);
}

void init_interrupts() {
	// Enable pin-change interrupt 1, for PCINT9 (PB1) to handle INTSW signal,
	// and for PCINT8 (PB0) to handle WASH signal.
	GIMSK |= _BV(PCIE1);
	PCMSK1 |= _BV(PCINT9) | _BV(PCINT8);
}

void init_adc() {
	// Set ADC reference to internal 2.2V (with AREF pin disconnected). Channel
	// selection in ADMUXA defaults to ADC0, which is what we want for pin PA0.
	ADMUXB |= _BV(REFS1);

	// Enable the ADC and start conversions, triggering automatically. Also
	// trigger an interrupt when each conversion completes. Clock prescaler to
	// 128.
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) | _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);

	// Set ADC auto-trigger to be timer 1 compare match A.
	ADCSRB |= _BV(ADTS2) | _BV(ADTS0);

	// Disable digital input on ADC0 to save power.
	DIDR0 |= _BV(ADC0D);
}

void init_timers() {
	// Set timer 0 clock source to be external (from T0, pin PA3), clocked on
	// rising edge. Interrupt to be triggered on counter overflow.
	TCCR0B |= _BV(CS00) | _BV(CS01) | _BV(CS02);
	TIMSK0 |= _BV(TOIE0);

	// Set timer 1 to run at 4Hz, and to generate an interrupt with compare
	// match A. The interrupt triggers sampling the ADC of int. mode volume
	// control, and also for gating VSS frequency measurement at 1Hz (every
	// fourth interrupt). All calculations assuming F_CPU = 8MHz.
	OCR1A = ((F_CPU / 256) / 4) - 1; // 4Hz
	TCCR1B |= _BV(WGM12) | _BV(CS12); // CTC mode with OCR1A as TOP value, clk/256 prescaler.
	TIMSK1 |= _BV(OCIE1A);

	// Set timer 2 to be 1kHz, generating an interrupt every millisecond, for
	// use as a system tick/clock of sorts.
	OCR2A = ((F_CPU / 256) / 1000) - 1;
	TCCR2B |= _BV(WGM22) | _BV(CS22); // CTC mode with OCR2A as TOP, clk/256 prescaler.
	TIMSK2 |= _BV(OCIE2A);
}

ISR(PCINT1_vect) {
	update_switch_states();
}

ISR(TIMER0_OVF_vect) {
	vss_overflows++;
}

ISR(TIMER1_COMPA_vect) {
	static uint8_t count = 4;

	// Update the current frequency of the VSS signal once per second - that is,
	// every 4th execution of this interrupt.
	if(--count == 0) {
		vss_frequency = (vss_overflows * 0xFF) + TCNT0;
		TCNT0 = 0;
		vss_overflows = 0;
		count = 4;
	}
}

ISR(TIMER2_COMPA_vect) {
	sys_timestamp++;
}

ISR(ADC_vect) {
	int_vol_adc_val = ADC;
	update_int_vol_state();
}

void print_word_array(const uint16_t * array, const size_t size) {
	for(size_t i = 0; i < size; i++) {
		printf_P(PSTR("%u = %u\n"), i, array[i]);
	}
}

bool validate_word_array_values_range(const uint16_t * array, const size_t size, const uint16_t min_val, const uint16_t max_val) {
	// Check all values are within the given range.
	for(size_t i = 0; i < size; i++) {
		if(array[i] < min_val || array[i] > max_val) {
			printf_P(PSTR("Invalid value (%u = %u); not in range %u-%u!\n"), i, array[i], min_val, max_val);
			return false;
		}
	}

	return true;
}

bool validate_word_array_values_ascending(const uint16_t * array, const size_t size) {
	// Can't do this unless array has at least two values, so just skip
	// validation and return as valid if it doesn't.
	if(size >= 2) {
		// Check all values are an ascending progression, each larger than the
		// previous value.
		for(size_t i = 1; i < size; i++) {
			if(array[i] <= array[i - 1]) {
				printf_P(PSTR("Invalid value pair (%u = %u, %u = %u); not ascending!\n"), i, array[i], i - 1, array[i - 1]);
				return false;
			}
		}
	}

	return true;
}

void config_load_and_validate() {
	printf_P(PSTR("Loading %u bytes of config data from EEPROM...\n"), sizeof(config_t));

	// Read the entire lot of configuration data from EEPROM in one go.
	eeprom_busy_wait();
	eeprom_read_block(&config, (const void *)CONFIG_EEPROM_ADDR, sizeof(config_t));

	printf_P(PSTR("Validating config values...\n"));

	// Now validate all the data read. If any are wrong, the default values are
	// used instead.
	config_validate_int_vol_thresholds();
	config_validate_int_vol_intervals();
	config_validate_wash_wipe_timeout();
	config_validate_wash_wipe_count();
	config_validate_speed_map_vss_thresholds();
	config_validate_speed_map_int_vol_settings();
}

void config_validate_int_vol_thresholds() {
	printf_P(PSTR("Int. vol. thresholds:\n"));

	bool load_defaults = !(
		validate_word_array_values_range(config.int_vol_thresholds, INTVOL_SETTINGS_COUNT, 1, 1023) &&
		validate_word_array_values_ascending(config.int_vol_thresholds, INTVOL_SETTINGS_COUNT)
	);

	if(load_defaults) {
		printf_P(PSTR("ERROR, using default\n"));
		memcpy_P(config.int_vol_thresholds, config_int_vol_thresholds_default, sizeof(config.int_vol_thresholds));
	} else {
		printf_P(PSTR("OK\n"));
	}

	print_word_array(config.int_vol_thresholds, INTVOL_SETTINGS_COUNT);
}

void config_validate_int_vol_intervals() {
	printf_P(PSTR("Int. vol. intervals:\n"));

	bool load_defaults = !(
		validate_word_array_values_range(config.int_vol_intervals, INTVOL_SETTINGS_COUNT, 1, 65534) &&
		validate_word_array_values_ascending(config.int_vol_intervals, INTVOL_SETTINGS_COUNT)
	);

	if(load_defaults) {
		printf_P(PSTR("ERROR, using default\n"));
		memcpy_P(config.int_vol_intervals, config_int_vol_intervals_default, sizeof(config.int_vol_intervals));
	} else {
		printf_P(PSTR("OK\n"));
	}

	print_word_array(config.int_vol_intervals, INTVOL_SETTINGS_COUNT);
}

void config_validate_wash_wipe_timeout() {
	printf_P(PSTR("Wash wipe timeout:\n"));

	// Check the value for empty EEPROM (0xFFFF) or zero value and use default
	// if either.
	if(config.wash_wipe_timeout == 0 || config.wash_wipe_timeout == 65535) {
		printf_P(PSTR("ERROR, using default\n"));
		config.wash_wipe_timeout = WASH_WIPE_TIMEOUT_EEPROM_DEFAULT;
	} else {
		printf_P(PSTR("OK\n"));
	}

	printf_P(PSTR("%u\n"), config.wash_wipe_timeout);
}

void config_validate_wash_wipe_count() {
	printf_P(PSTR("Wash wipe count:\n"));

	// Check the value for empty EEPROM (0xFF) or zero value and use default if
	// either.
	if(config.wash_wipe_count == 0 || config.wash_wipe_count == 255) {
		printf_P(PSTR("ERROR, using default\n"));
		config.wash_wipe_count = WASH_WIPE_COUNT_EEPROM_DEFAULT;
	} else {
		printf_P(PSTR("OK\n"));
	}

	printf_P(PSTR("%u\n"), config.wash_wipe_count);
}

void config_validate_speed_map_vss_thresholds() {
	printf_P(PSTR("Speed map VSS thresholds:\n"));

	bool load_defaults = !(
		validate_word_array_values_range(config.speed_map_vss_thresholds, SPEED_MAP_VSS_THRESHOLD_COUNT, 0, 250) &&
		validate_word_array_values_ascending(config.speed_map_vss_thresholds, SPEED_MAP_VSS_THRESHOLD_COUNT)
	);

	if(load_defaults) {
		printf_P(PSTR("ERROR, using default\n"));
		memcpy_P(config.speed_map_vss_thresholds, config_speed_map_vss_thresholds_default, sizeof(config.speed_map_vss_thresholds));
	} else {
		printf_P(PSTR("OK\n"));
	}

	print_word_array(config.speed_map_vss_thresholds, SPEED_MAP_VSS_THRESHOLD_COUNT);
}

void config_validate_speed_map_int_vol_settings() {
	bool load_defaults = false;

	printf_P(PSTR("Speed map int. vol. settings:\n"));

	for(size_t i = 0; i < INTVOL_SETTINGS_COUNT; i++) {
		for(size_t j = 0; j < SPEED_MAP_VSS_THRESHOLD_COUNT; j++) {
			// Make sure value is within expected range. For empty EEPROM,
			// values will be 255 (0xFF), so this'll catch that.
			if(config.speed_map_int_vol_settings[i][j] >= INTVOL_SETTINGS_COUNT) {
				printf_P(PSTR("Invalid value (%u,%u = %u); out of range!\n"), i, j, config.speed_map_int_vol_settings[i][j]);
				load_defaults = true;
				break;
			}
		}
		if(load_defaults) break;
	}

	if(load_defaults) {
		printf_P(PSTR("ERROR, using default\n"));
		memcpy_P(config.speed_map_int_vol_settings, config_speed_map_int_vol_settings_default, sizeof(config.speed_map_int_vol_settings));
	} else {
		printf_P(PSTR("OK\n"));
	}

	// Print the contents of the two-dimensional array.
	for(size_t i = 0; i < INTVOL_SETTINGS_COUNT; i++) {
		printf_P(PSTR("%u = "), i);
		for(size_t j = 0; j < SPEED_MAP_VSS_THRESHOLD_COUNT; j++) {
			if(j > 0) printf_P(PSTR(", "));
			printf_P(PSTR("%u"), config.speed_map_int_vol_settings[i][j]);
		}
		printf_P(PSTR("\n"));
	}
}

void check_vss() {
#ifdef SPEED_MAP_ENABLED
	// Check to see if the VSS is high due to pull-up to VCC being in-circuit
	// because connector pins are bridged. If high, flag that speed mapping
	// should be disabled.
	speed_map_disabled = bit_is_set(PINA, PINA3);
	printf_P(PSTR("VSS input is %S; speed-mapping %S\n"), (speed_map_disabled ? PSTR("HIGH") : PSTR("LOW")), (speed_map_disabled ? PSTR("disabled") : PSTR("enabled")));
#else
	speed_map_disabled = true;
#endif
}

void update_switch_states() {
	int_sw = bit_is_set(PINB, PINB1);
	wash_sw = bit_is_set(PINB, PINB0);
}

void update_int_vol_state() {
	// Note: we don't need to worry about atomically accessing int_vol_* and
	// vss_frequency global vars, as this routine is only ever called in
	// contexts where interrupts are already globally disabled.

	// Set a safe default of shortest interval, just to be safe (in case for
	// some bizarre reason we're unable to determine setting from reading ADC
	// value below).
	int_vol_setting_commanded = 0;

	// Work out commanded intermittent mode volume setting by comparing to
	// threshold values.
	for(size_t i = 0; i < INTVOL_SETTINGS_COUNT; i++) {
		if(int_vol_adc_val <= config.int_vol_thresholds[i]) {
			int_vol_setting_commanded = i;
			break;
		}
	}

	// Default the actual setting to equal the commanded.
	int_vol_setting_actual = int_vol_setting_commanded;

	if(!speed_map_disabled) {
		// Given the current VSS frequency, look up which speed map threshold it
		// falls below (or equal to). Once found, using the speed map table,
		// look up what volume setting we should actually be using.
		for(size_t i = 0; i < SPEED_MAP_VSS_THRESHOLD_COUNT; i++) {
			if(vss_frequency <= config.speed_map_vss_thresholds[i]) {
				int_vol_setting_actual = config.speed_map_int_vol_settings[int_vol_setting_commanded][i];
				break;
			}
		}
	}

	// Finally, look up the interval period for the eventual volume setting.
	int_vol_interval = config.int_vol_intervals[int_vol_setting_actual];
}

void wipe(const bool immediate) {
	timestamp_t interval_begin, interval_elapsed = 0;
	bool actuate = false;

	printf_P(PSTR("Intermittent wipe triggered!\n"));

	if(immediate) {
		printf_P(PSTR("Wiping immediately\n"));

		actuate = true;
	} else {
		printf_P(PSTR("Waiting for interval...\n"));

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			interval_begin = sys_timestamp;
		}

		// Wait for the interval period only as long as intermittent mode switch
		// is on and wash switch is off. If the wash switch is turned on, we
		// will immediately exit (and subsequently attend to that action).
		while(int_sw && !wash_sw) {
			// Note: interval may change during this waiting period. So, if user
			// reduces int. vol. in middle of wait, we shall stop waiting if the
			// new interval period has already elapsed. Opposite also applies.

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
				interval_elapsed = sys_timestamp - interval_begin;
				actuate = (interval_elapsed >= int_vol_interval);
			}

			if(actuate) break;
		}

		printf_P(PSTR("Stopped waiting; %ld ms elapsed\n"), interval_elapsed);
	}

	if(actuate) wipe_actuate(1);
}

void wipe_actuate(uint8_t count) {
	if(count == 0) return;

	if(count > 1) printf_P(PSTR("Wiping %u times...\n"), count);

	// Turn relay on.
	relay_on();
	printf_P(PSTR("Wiper relay ON\n"));

	while(count-- > 0) {
		// Wait for auto-stop input signal to be high, but only wait for a
		// certain period until we time-out and assume something is wrong, in
		// which case turn off the relay and quit.
		printf_P(PSTR("Waiting for AUTOSTOP = ON (%u ms timeout)...\n"), WIPE_AUTOSTOP_HIGH_TIMEOUT_MS);
		if(!wipe_autostop_wait(true, WIPE_AUTOSTOP_HIGH_TIMEOUT_MS)) {
			printf_P(PSTR("Timed out!\n"));
			break;
		}
		printf_P(PSTR("Got AUTOSTOP = ON!\n"));

		// Wait for auto-stop input signal to be low, but again only wait for a
		// certain period until we time-out and assume something is wrong, in
		// which case turn off the relay and quit.
		printf_P(PSTR("Waiting for AUTOSTOP = OFF (%u ms timeout)...\n"), WIPE_AUTOSTOP_LOW_TIMEOUT_MS);
		if(!wipe_autostop_wait(false, WIPE_AUTOSTOP_LOW_TIMEOUT_MS)) {
			printf_P(PSTR("Timed out!\n"));
			break;
		}
		printf_P(PSTR("Got AUTOSTOP = OFF!\n"));

		if(count > 0) printf_P(PSTR("%u wipes remaining\n"), count);
	}

	// Turn off the relay.
	relay_off();
	printf_P(PSTR("Wiper relay OFF\n"));
}

bool wipe_autostop_wait(const bool wait_for_state, const uint16_t timeout) {
	timestamp_t wait_begin, wait_elapsed = 0;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		wait_begin = sys_timestamp;
	}

	// If we are to wait for a high state, loop while the auto-stop input signal
	// is low, and for the opposite, loop while the signal is high.
	while(wait_for_state ? bit_is_clear(PINB, PINB2) : bit_is_set(PINB, PINB2)) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			wait_elapsed = sys_timestamp - wait_begin;
		}

		if(wait_elapsed >= timeout) return false;
	}

	return true;
}

void wash() {
	timestamp_t wash_begin, wash_elapsed = 0;

	// Make a note of timestamp at which wash was initially triggered.
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		wash_begin = sys_timestamp;
	}

	printf_P(PSTR("Wash triggered at %ld; wipe timeout = %u ms, count = %u\n"), wash_begin, config.wash_wipe_timeout, config.wash_wipe_count);

	// As long as the wash switch is on, see if the timeout period has elapsed,
	// and if it has, trigger the wipers once. If the wash switch is released
	// before the timeout, we just return anyway.
	while(wash_sw) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			wash_elapsed = sys_timestamp - wash_begin;
		}

		if(wash_elapsed >= config.wash_wipe_timeout) {
			printf_P(PSTR("Wash ON for %ld ms - wiping!\n"), wash_elapsed);
			wipe_actuate(config.wash_wipe_count);
			break;
		}
	}
}

void standby() {
	// Wait for any pending UART transmission to finish.
	uart_flush();

	// Set sleep to full power down. Only external/pin-change interrupts or the
	// watchdog timer can wake the CPU.	Then enable sleep and enter sleep mode!
	// Power down all peripheral modules before to save power, then re-enable
	// only the ones we're using.
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	power_disable_all();
	sleep_mode();
	power_enable_used();
}
