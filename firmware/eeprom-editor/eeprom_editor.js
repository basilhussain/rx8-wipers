/*
function buf2hex(buffer) {
	// Take each byte from buffer and convert to hex (padding with
	// zero character to 2 digits) and return as an uppercase string.
	return Array.prototype.map.call(new Uint8Array(buffer), function(val) {
		return ('00' + val.toString(16)).slice(-2);
	}).join('').toUpperCase();
}
*/

function hex(val, len) {
	return val.toString(16).padStart(len, '0').toUpperCase();
}

function intel_hex_encode(data_buffer) {
	const record_size = 32;
	const data_bytes = new Uint8Array(data_buffer);
	
	var output = '';
	
	for(let addr = 0, remain = data_bytes.length; addr < data_bytes.length; addr += record_size, remain -= record_size) {
		// Make byte count the number of remaining bytes if less than a full record.
		const record_byte_count = (remain < record_size ? remain : record_size);
		const record_buff = new ArrayBuffer(5 + record_byte_count);
		const record_view = new DataView(record_buff);
		const record_all = new Uint8Array(record_buff);
		const record_data = new Uint8Array(record_buff, 4, record_byte_count);
		
		// Begin a new record, specifying byte count per record,
		// address and record type (0 = data).
		record_view.setUint8(0, record_byte_count);
		record_view.setUint16(1, addr);
		record_view.setUint8(3, 0);
		
		// Copy the data out of the given buffer and into the record's.
		record_data.set(data_bytes.slice(addr, addr + record_byte_count));
		
		// Calculate the checksum from all the record's bytes (except
		// last checksum byte, naturally). Checksum is the two's-complement
		// of the LSB of the sum of all preceding bytes in the record.
		let checksum = record_all.slice(0, -1).reduce(function(sum, val) {
			return sum + val;
		}, 0);
		checksum = ~checksum + 1;
		record_view.setUint8(4 + record_byte_count, checksum);
		
		// Finally, add the record in hex format to the output.
		output += ':';
		record_all.forEach(function(val) {
			output += hex(val, 2);
		});
		output += "\r\n";
	}
	
	// Add the final end-of-file (EOF) record. Is of fixed value, so hard-coded.
	output += ":00000001FF\r\n";
	
	return output;
}

function calculate() {
	// Default EEPROM contents, with all int. vol. threshold bytes set to 0xFF.
	const EEPROM_SIZE = 512;
	const EEPROM_DEFAULT = [
		0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0x08, 0x07, 0x98, 0x08, 0xAC, 0x0D, 0xF8, 0x11, 0xE0, 0x15, 0x90, 0x1A, 0x40,
		0x1F, 0x8C, 0x23, 0xE4, 0x25, 0xEE, 0x02, 0x03, 0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x0F, 0x00,
		0x14, 0x00, 0x08, 0x05, 0x02, 0x01, 0x00, 0x08, 0x06, 0x04, 0x03, 0x02, 0x08, 0x06, 0x05, 0x04,
		0x03, 0x08, 0x07, 0x06, 0x05, 0x04, 0x08, 0x07, 0x06, 0x05, 0x04, 0x08, 0x08, 0x07, 0x06, 0x06,
		0x08, 0x08, 0x07, 0x07, 0x06, 0x08, 0x08, 0x08, 0x07, 0x07, 0x08, 0x08, 0x08, 0x08, 0x08
	];
	const EEPROM_EMPTY_FILL = 0xFF;

	var eeprom = new ArrayBuffer(EEPROM_SIZE);
	var eeprom_default = new Uint8Array(eeprom, 0, EEPROM_DEFAULT.length);
	var int_vol_thresholds = new DataView(eeprom, 1, 18);
	var empty = new Uint8Array(eeprom, EEPROM_DEFAULT.length);
	
	// Load the default EEPROM data into the buffer.
	eeprom_default.set(EEPROM_DEFAULT);
	
	// Calculate the int. vol. threshold values from the form data and
	// fill that into the buffer. Values are written in little-endian format.
	var thresholds = calculate_int_vol_thresholds();
	thresholds.forEach(function(val, idx) {
		int_vol_thresholds.setUint16(idx * Uint16Array.BYTES_PER_ELEMENT, val, true);
	});
	
	// Finally fill up the remaining empty space.
	empty.fill(EEPROM_EMPTY_FILL);
	
	return intel_hex_encode(eeprom);
}

function calculate_int_vol_thresholds() {
	const INTVOL_SETTINGS_COUNT = 9;
	const VCC = 5;
	const DIVIDER_PAIR_RES = 62;
	const ADC_REF = 2.2;
	const ADC_RESOLUTION = 1024;

	var data = [];
	
	for(let i = 0; i < INTVOL_SETTINGS_COUNT; i++) {
		let min_res = parseFloat(document.getElementById("setting_" + i + "_min_res").value);
		let max_res = parseFloat(document.getElementById("setting_" + i + "_max_res").value);
		
		if(!isNaN(min_res) && !isNaN(max_res)) {
			data[i] = { 'min_res': min_res, 'max_res': max_res };
		}
	}
	
	// Calculate the min and max resulting input voltages according to the resistances given.
	for(let i = 0; i < INTVOL_SETTINGS_COUNT; i++) {
		data[i]['min_volts'] = (data[i]['min_res'] / (DIVIDER_PAIR_RES + data[i]['min_res'])) * VCC;
		data[i]['max_volts'] = (data[i]['max_res'] / (DIVIDER_PAIR_RES + data[i]['max_res'])) * VCC;
	}
	
	// Calculate the min and max ADC values from the previous voltages.
	// Cap the ADC values at the maximum value possible according to the ADC resolution.
	for(let i = 0; i < INTVOL_SETTINGS_COUNT; i++) {
		data[i]['min_adc'] = Math.min(Math.floor((data[i]['min_volts'] / ADC_REF) * ADC_RESOLUTION), ADC_RESOLUTION - 1);
		data[i]['max_adc'] = Math.min(Math.floor((data[i]['max_volts'] / ADC_REF) * ADC_RESOLUTION), ADC_RESOLUTION - 1);
	}
	
	// Calculate the ADC value upper threshold for each setting. The threshold for
	// setting 8 is always the maximum possible according to ADC resolution.
	for(let i = 0; i < INTVOL_SETTINGS_COUNT - 1; i++) {
		data[i]['threshold'] = Math.round(data[i + 1]['min_adc'] - ((data[i + 1]['min_adc'] - data[i]['max_adc']) / 2));
	}
	data[INTVOL_SETTINGS_COUNT - 1]['threshold'] = ADC_RESOLUTION - 1;
	
	// Return an array consisting only of the thresholds.
	return data.map(function(item) {
		return item['threshold'];
	});
}

document.addEventListener("readystatechange", function(event) {
	if(document.readyState === "interactive") {
		var out = document.getElementById("out");
		var form = document.getElementById("form");

		form.addEventListener("submit", function(event) {
			out.value = calculate();
			event.preventDefault();
		});
	}
});
