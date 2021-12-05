/**
 *
 * HX711 library for Mbed.
 * Arduino source: https://github.com/bogde/HX711
 *
**/
#include "HX711_mbed.h"

// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t shiftInSlow(DigitalOut* CLK, DigitalInOut* DATA) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        CLK->write(1);
        wait_us(1);
        value |= (DATA->read() << (7 - i));
        CLK->write(0);
        wait_us(1);
    }

    return value;
}

HX711::HX711() {
}

HX711::~HX711() {
}

void HX711::begin(DigitalOut* clk, DigitalInOut* data, uint8_t gain) {
	CLK = clk;
	DATA = data;
	set_gain(gain);
}

bool HX711::is_ready() {
	return (DATA->read() == 0);
}

void HX711::set_gain(uint8_t gain) {
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}

}

long HX711::read() {

	// Wait for the chip to become ready.
	wait_ready_timeout();

	// Define structures for reading data into.
	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  This block disables interrupts during 
    // the sequence and then restores the interrupt mask to its previous state after the
    // sequence completes, insuring that the entire read-and-gain-set sequence is not
    // interrupted.

	// Disable interrupts.
	__disable_irq();

	// Pulse the clock pin 24 times to read the data.
	data[2] = shiftInSlow(CLK, DATA);
	data[1] = shiftInSlow(CLK, DATA);
	data[0] = shiftInSlow(CLK, DATA);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < GAIN; i++) {
        CLK->write(1);
        wait_us(1);
		CLK->write(0);
        wait_us(1);
	}

	// Enable interrupts again.
	__enable_irq();

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );

	return static_cast<long>(value);
}

void HX711::wait_ready(unsigned long delay_ms) {
	// Wait for the chip to become ready.
	// This is a blocking implementation and will
	// halt the sketch until a load cell is connected.
	while (!is_ready()) {
		// Probably will do no harm on AVR but will feed the Watchdog Timer (WDT) on ESP.
		// https://github.com/bogde/HX711/issues/73
        ThisThread::sleep_for(chrono::milliseconds(delay_ms));
	}
}

bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	// Wait for the chip to become ready by
	// retrying for a specified amount of attempts.
	// https://github.com/bogde/HX711/issues/76
	int count = 0;
	while (count < retries) {
		if (is_ready()) {
			return true;
		}
        ThisThread::sleep_for(chrono::milliseconds(delay_ms));
		count++;
	}
	return false;
}

bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	// Wait for the chip to become ready until timeout.
	// https://github.com/bogde/HX711/pull/96
    Timer t;
	t.start();
	while (timer_read_ms(t) < timeout) {
		if (is_ready()) {
			return true;
		}
        ThisThread::sleep_for(chrono::milliseconds(delay_ms));
	}
	return false;
}

long HX711::read_average(uint8_t times) {
	long sum = 0;
	for (uint8_t i = 0; i < times; i++) {
		sum += read();
	}
	return sum / times;
}

double HX711::get_value(uint8_t times) {
	return read_average(times) - OFFSET;
}

float HX711::get_units(uint8_t times) {
	return get_value(times) / SCALE;
}

void HX711::tare(uint8_t times) {
	double sum = read_average(times);
	set_offset(sum);
}

void HX711::set_scale(float scale) {
	SCALE = scale;
}

float HX711::get_scale() {
	return SCALE;
}

void HX711::set_offset(long offset) {
	OFFSET = offset;
}

long HX711::get_offset() {
	return OFFSET;
}

void HX711::power_down() {
    CLK->write(0);
    CLK->write(1);
}

void HX711::power_up() {
    CLK->write(0);
}
