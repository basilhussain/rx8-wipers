This is the firmware code for the speed-sensitive windscreen wiper upgrade for Mazda RX-8s.

The code is licenced under the BSD 3-Clause Licence. See the LICENSE.txt file for full licence text.

Code was developed using Atmel Studio 7, and project files are included.

The following pre-processor symbols are defined in the project properties and affect the code in various ways.

* F_CPU - CPU frequency, in Hz; delay timings and UART baud rate are reliant on this being correctly specified.
* BAUD - UART baud rate, in bits per second.
* BAUD_TOL - UART baud rate tolerance, in percent
* UART_IMPL_BLOCKING - use blocking UART implementation (other option is UART_IMPL_INTERRUPT, but that code is currently broken, so don't use).

For values required for fuse programming of the ATtiny chip, see the fuses.txt file.

To formulate an EEPROM data file containing calibration data, etc., an HTML/JS-based tool is provided. Load the index.html file in the eeprom_editor subdirectory in a web browser. Enter wiper stalk resistance measurements and click the button. Save the output Intel Hex-format data to a .hex file. This can then be used to program the chip's EEPROM.

Note for AVRdude users: at time of writing, the latest release version of AVRdude, 6.3, does not support the ATtiny841. In order to program the chip, please use the included avrdude_attiny_441_841.conf file. You will need to specify it with an additional "-C" command line argument, with the filename prefixed with a "+" (e.g. "-C +avrdude_attiny_441_841.conf"). The part type can then be specified with a "-p" argument of "t841".
