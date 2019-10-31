/*
 * Title   : mePed v2 Robot
 * Hardware: ATmega328P @ 16 MHz, HC-SR04 Ultrasonic Ranging Module
 *           SG90 Servos, 1838 IR Receiver, QMC5883L Magnetic Sensor
 *           and GL5528 LDRs
 * Created : 9-11-2018 19:51:35
 * Author  : Tim Dorssers
 *
 * The quadruped robot uses the HC-SR04 module as eyes and it uses the LDRs to
 * sense light. The HC-SR04 module is connected to PCINT10 and uses the 8-bit
 * Timer0 interrupt to measure RTT. The LDRs are connected to ADC0 and ADC1.
 * When an obstacle is detected within 30 cm distance, the robot turns into the
 * brightest direction, using the magnetic heading to turn at least 75 degrees.
 * When an obstacle is detected ahead within 15 cm distance, the robot reverses
 * until no obstacle is detected within 30 cm distance and then turns to the
 * brightest direction. The walking speed, robot height and leg positions are
 * adjustable.
 * Eight servos are connected to PORTD and PORTB and are driven by a software
 * PWM implementation using the 16-bit Timer1 output compare interrupt.
 * The IR remote receiver uses the 8-bit Timer2 interrupt to collect data.
 * Hardware I2C is used to read out the QMC5883L sensor. The raw readings are
 * converted to a heading in degrees. Automatic scaling and centering data is
 * stored in EEPROM. Debugging data is sent to the hardware UART.
 * The robot can be controlled with an IR remote control and RS232 or BT.
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/sfr_defs.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdbool.h>
#include "uart.h"
#include "servo.h"
#include "irremote.h"
#include "i2cmaster.h"

// Globals
char buffer[12];
typedef enum {FORWARD, TURN_RIGHT, BACKWARD, TURN_LEFT} state_t;
typedef enum {NEWLINE, COMMA} dump_t;
decode_results results;
uint16_t EEMEM nv_magic;

uint8_t spd = 10; // Speed of walking motion
int8_t trim = 15; // Leg position calibration
int8_t high = 0;  // How high the robot is standing
uint8_t s11 = 90; // Front Left Pivot Servo
uint8_t s12 = 90; // Front Left Lift Servo
uint8_t s21 = 90; // Back Left Pivot Servo
uint8_t s22 = 90; // Back Left Lift Servo
uint8_t s31 = 90; // Back Right Pivot Servo
uint8_t s32 = 90; // Back Right Lift Servo
uint8_t s41 = 90; // Front Right Pivot Servo
uint8_t s42 = 90; // Front Right Lift Servo

// set servo positions and speeds needed to walk forward one step
// LFP, LBP, RBP, RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4
const uint8_t srv_forward[] PROGMEM = {
	180, 0,  120, 60, 100, 90, 90, 100, 1, 3, 1, 1,
	 90, 30, 90,  30, 60,  90, 90, 100, 3, 1, 1, 1,
	 90, 30, 90,  30, 100, 90, 90, 100, 3, 1, 1, 1,
	120, 60, 180, 0,  100, 90, 60, 100, 1, 1, 3, 1,
	120, 60, 180, 0,  100, 90, 90, 100, 1, 1, 3, 1,
	150, 90, 150, 90, 100, 90, 90, 60,  1, 1, 1, 3,
	150, 90, 150, 90, 100, 90, 90, 100, 1, 1, 1, 3,
	180, 0,  120, 60, 100, 60, 90, 100, 1, 3, 1, 1};
// set servo positions and speeds needed to walk backward one step
// LFP, LBP, RBP, RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4
const uint8_t srv_back[] PROGMEM = {
	180, 0,  120, 60, 100, 90, 90, 100, 3, 1, 1, 1,
	150, 90, 150, 90, 100, 75, 90, 100, 1, 3, 1, 1,
	150, 90, 150, 90, 100, 90, 90, 100, 1, 3, 1, 1,
	120, 60, 180, 0,  100, 90, 90, 60,  1, 1, 1, 3,
	120, 60, 180, 0,  100, 90, 90, 100, 1, 1, 1, 3,
	90,  30, 90,  30, 100, 90, 75, 100, 1, 1, 3, 1,
	90,  30, 90,  30, 100, 90, 90, 100, 1, 1, 3, 1,
	180, 0,  120, 60, 60,  90, 90, 100, 3, 1, 1, 1};
// set servo positions and speeds needed to turn left one step
// LFP, LBP, RBP, RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4
const uint8_t srv_left[] PROGMEM = {
	150, 90, 90,  30, 100, 60, 90, 100, 1, 3, 1, 1,
	150, 90, 90,  30, 100, 90, 90, 100, 1, 3, 1, 1,
	120, 60, 180, 0,  100, 90, 60, 100, 1, 1, 3, 1,
	120, 60, 180, 0,  100, 90, 90, 100, 1, 1, 3, 1,
	90,  30, 150, 90, 100, 90, 90, 60,  1, 1, 1, 3,
	90,  30, 150, 90, 100, 90, 90, 100, 1, 1, 1, 3,
	180, 0,  120, 60, 60,  90, 90, 100, 3, 1, 1, 1,
	180, 0,  120, 60, 100, 90, 90, 90,  3, 1, 1, 1};
// set servo positions and speeds needed to turn right one step
// LFP, LBP, RBP, RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4
const uint8_t srv_right[] PROGMEM = {
	 90, 30, 150, 90, 60,  90, 90, 100, 3, 1, 1, 1,
	 90, 30, 150, 90, 100, 90, 90, 100, 3, 1, 1, 1,
	120, 60, 180, 0,  100, 90, 90, 60,  1, 1, 1, 3,
	120, 60, 180, 0,  100, 90, 90, 100, 1, 1, 1, 3,
	150, 90, 90,  30, 100, 90, 60, 100, 1, 1, 3, 1,
	150, 90, 90,  30, 100, 90, 90, 100, 1, 1, 3, 1,
	180, 0,  120, 60, 100, 60, 90, 100, 1, 3, 1, 1,
	180, 0,  120, 60, 100, 90, 90, 100, 1, 3, 1, 1};
// miscellaneous servo positions and speeds
// LFP, LBP, RBP, RFP, LFL, LBL, RBL, RFL, S1, S2, S3, S4
const uint8_t srv_center[] PROGMEM = {
	90, 90, 90, 90,  90,  90,  90,  90, 3, 3, 3, 3};
const uint8_t srv_lean_left[] PROGMEM = {
	90, 90, 90, 90,  60,  60, 100, 100, 3, 3, 3, 3};
const uint8_t srv_lean_right[] PROGMEM = {
	90, 90, 90, 90, 100, 100,  60,  60, 3, 3, 3, 3};
const uint8_t srv_bow[] PROGMEM = {
	90, 90, 90, 90,  60,  90,  90,  60, 1, 1, 1, 1,
	90, 90, 90, 90,  90,  90,  90,  90, 1, 1, 1, 1};

//Function macros
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define dump_value_P(__s, __v, __p) dump_value_p(PSTR(__s), __v, __p)

// Magnetic sensor
#define MAGNETO_ADDR (0x0d << 1)  // I2C bus address
int16_t xlow = 0, ylow = 0, xhigh = 0, yhigh = 0;
uint16_t EEMEM nv_xlow, nv_xhigh, nv_ylow, nv_yhigh;

// Ultrasonic ranging sensor
#define MAX_DISTANCE  300      // sets maximum usable sensor measuring distance
#define CM_ECHO_TIME  (F_CPU / 17013)    // Speed of sound in cm divided by two
#define MAX_ECHO_TIME (CM_ECHO_TIME * MAX_DISTANCE)  // Maximum sensor distance
uint32_t countTimer0;
volatile bool echoDone;

// Get distance in cm from HC-SR04
uint16_t ping_cm(void) {
	static uint16_t lastPing;
	
	PCICR |= _BV(PCIE1);    // Pin Change Interrupt Enable 1
	PCMSK1 |= _BV(PCINT10); // Mask PCINT10
	echoDone = false;       // set echo flag
	countTimer0 = 0;        // reset counter
	// send 10us trigger pulse
	PORTC &= ~_BV(PC3);
	_delay_us(4);
	PORTC |= _BV(PC3);
	_delay_us(10);
	PORTC &= ~_BV(PC3);
	// Previous ping hasn't finished, abort.
	if (bit_is_set(PINC, PINC2))
		return lastPing;
	// loop till echo pin goes low
	while(!echoDone);
	// disable interrupt
	PCICR &= ~_BV(PCIE1);
	PCMSK1 &= ~_BV(PCINT10);
	// calculate distance
	lastPing = countTimer0 / CM_ECHO_TIME;
	return lastPing;
}

// Timer0 interrupt fires F_CPU / 256 times per second for echo pulse time measurement
ISR(TIMER0_OVF_vect) {
	countTimer0 += 255;
	if (countTimer0 > MAX_ECHO_TIME) {
		TCCR0B = 0;           // Timer0 Stopped
		countTimer0 += TCNT0; // calculate time passed
		TCNT0 = 0;            // Reset Timer0
		echoDone = true;      // set flag
	}
}

// HC-SR04 echo pin interrupt
ISR(PCINT1_vect){
	if (bit_is_set(PINC, PC2)) {
		// rising edge
		TCCR0B = _BV(CS00);   // Timer0 Clock Select No Prescaling
	} else {
		// falling edge
		TCCR0B = 0;           // Timer0 Stopped
		countTimer0 += TCNT0; // calculate time passed
		TCNT0 = 0;            // Reset Timer0
		echoDone = true;      // set flag
	}
}

// Initialize HC-SR04
static void init_sonar(void) {
	DDRC |= _BV(PC3);     // Trigger pin as output
	PORTC |= _BV(PC2);    // Pull up echo pin
	TIMSK0 |= _BV(TOIE0); // Timer0 Overflow Interrupt Enable
}

// Initialize ADC
static void init_adc(void) {
	// AVCC with external capacitor at AREF pin and ADC Left Adjust Result
	ADMUX = _BV(ADLAR) | _BV(REFS0);
	// ADC prescaler of 128 and enable ADC
	ADCSRA = _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0) | _BV(ADEN);
}

// Read single conversion from given ADC channel
uint8_t read_adc(uint8_t ch) {
	ADMUX = (ADMUX & 0xf0) | (ch & 0x0f);	// select channel
	ADCSRA |= _BV(ADSC);					// start conversion
	while (ADCSRA & _BV(ADSC));				// wait until conversion complete
	return ADCH;
}

// Dump long to UART with label string stored in progmem
void dump_value_p(const char *progmem_s, long val, dump_t postfix) {
	ltoa(val, buffer, 10);
	uart_puts_p(progmem_s);
	uart_puts(buffer);
	switch (postfix) {
		case COMMA:
			uart_puts_P(", ");
			break;
		default:
			uart_puts_P("\r\n");
	}
}

// Initialize QMC5883L
static void magneto_init(void) {
	i2c_start(MAGNETO_ADDR | I2C_WRITE);
	i2c_write(0x0b); // SET/RESET period register
	i2c_write(0x01); // Recommended value
	i2c_stop();
	i2c_start(MAGNETO_ADDR | I2C_WRITE);
	i2c_write(0x09); // Control register 1
	i2c_write(0x1d); // Continuous Mode, 200Hz ODR, 8G RNG, 512 OSR
	i2c_stop();
}

// Get raw reading from QMC5883L
static void magneto_getraw(int16_t *x, int16_t *y, int16_t *z) {
	if (!i2c_start(MAGNETO_ADDR | I2C_WRITE)) {
		if (!i2c_write(0x00)) {
			i2c_stop();
			if (!i2c_start(MAGNETO_ADDR | I2C_READ)) {
				/* Read 16 bit x, y, z value (2's complement form) */
				*x = (int16_t)i2c_readAck() | ((int16_t)i2c_readAck() << 8);
				*y = (int16_t)i2c_readAck() | ((int16_t)i2c_readAck() << 8);
				*z = (int16_t)i2c_readAck() | ((int16_t)i2c_readNak() << 8);
			}
		}
	}
	i2c_stop();
}

static void init_eeprom(void) {
	if (eeprom_read_word(&nv_magic) == 0xaa55) {
		xlow = eeprom_read_word(&nv_xlow);
		xhigh = eeprom_read_word(&nv_xhigh);
		ylow = eeprom_read_word(&nv_ylow);
		yhigh = eeprom_read_word(&nv_yhigh);
	} else
		eeprom_update_word(&nv_magic, 0xaa55);
}

void update_eeprom(void) {
	eeprom_update_word(&nv_xlow, xlow);
	eeprom_update_word(&nv_xhigh, xhigh);
	eeprom_update_word(&nv_ylow, ylow);
	eeprom_update_word(&nv_yhigh, yhigh);
}

void reset_bounds(void) {
	xlow = xhigh = ylow = yhigh = 0;
	uart_puts_P("zeroed\r\n");
	update_eeprom();
}

// Get heading from QMC5883L
uint16_t magneto_heading(void) {
	int16_t x = 0, y = 0, z = 0, heading;
	double xscaled, yscaled;
	
	magneto_getraw(&x, &y, &z);
	if (xlow == 0 && xhigh == 0)
		xlow = xhigh = x;
	if (ylow == 0 && yhigh == 0)
		ylow = yhigh = y;
	// Update the observed boundaries of the measurements
	xlow = min(x, xlow);
	xhigh = max(x, xhigh);
	ylow = min(y, ylow);
	yhigh = max(y, yhigh);
	update_eeprom();
	// Recenter the measurement by subtracting the average
	x -= (xhigh + xlow) / 2;
	y -= (yhigh + ylow) / 2;
	// Rescale the measurement to the range observed
	xscaled = (double)x / (xhigh - xlow);
	yscaled = (double)y / (yhigh - ylow);
	// Calculate heading and convert to degrees
	heading = 180.0 * atan2(yscaled, xscaled) / M_PI;
	if (heading <= 0)
		heading += 360;
	return heading;
}

// Write Servo Values
void set_servos(void) {
	// Write Pivot Servo Values
	srvo1a_setpr(1, s11);
	srvo1a_setpr(3, s21);
	srvo1a_setpr(5, s31);
	srvo1b_setpr(1, s41);
	// Write Lift Servo Values
	srvo1a_setpr(2, s12);
	srvo1a_setpr(4, s22);
	srvo1a_setpr(6, s32);
	srvo1b_setpr(2, s42);
}

// Move servo positions to values stored in progmem array
void srv_move(const uint8_t *ptr) {
	uint8_t p11 = pgm_read_byte(ptr++); // Front Left Pivot Servo
	uint8_t p21 = pgm_read_byte(ptr++); // Back Left Pivot Servo
	uint8_t p31 = pgm_read_byte(ptr++); // Back Right Pivot Servo
	uint8_t p41 = pgm_read_byte(ptr++); // Front Right Pivot Servo
	uint8_t p12 = pgm_read_byte(ptr++); // Front Left Lift Servo
	uint8_t p22 = pgm_read_byte(ptr++); // Back Left Lift Servo
	uint8_t p32 = pgm_read_byte(ptr++); // Back Right Lift Servo
	uint8_t p42 = pgm_read_byte(ptr++); // Front Right Lift Servo
	uint8_t sp1 = pgm_read_byte(ptr++); // Speed 1
	uint8_t sp2 = pgm_read_byte(ptr++); // Speed 2
	uint8_t sp3 = pgm_read_byte(ptr++); // Speed 3
	uint8_t sp4 = pgm_read_byte(ptr++); // Speed 4
	
	// calculation of points
	p11 -= trim;
	p21 += trim;
	p31 -= trim;
	p41 += trim;
	// height adjustment
	p12 += high;
	p22 += high;
	p32 += high;
	p42 += high;
	while ((s11 != p11) || (s21 != p21) || (s31 != p31) || (s41 != p41) || (s12 != p12) || (s22 != p22) || (s32 != p32) || (s42 != p42)) {
		// Front Left Pivot Servo
		if (s11 < p11)            // if servo position is less than programmed position
			s11 = min(s11 + sp1, p11); // set servo position equal to servo position plus speed constant
		if (s11 > p11)            // if servo position is greater than programmed position
			s11 = max(s11 - sp1, p11); // set servo position equal to servo position minus speed constant
		// Back Left Pivot Servo
		if (s21 < p21)
			s21 = min(s21 + sp2, p21);
		if (s21 > p21)
			s21 = max(s21 - sp2, p21);
		// Back Right Pivot Servo
		if (s31 < p31)
			s31 = min(s31 + sp3, p31);
		if (s31 > p31)
			s31 = max(s31 - sp3, p31);
		// Front Right Pivot Servo
		if (s41 < p41)
			s41 = min(s41 + sp4, p41);
		if (s41 > p41)
			s41 = max(s41 - sp4, p41);
		// Front Left Lift Servo
		if (s12 < p12)
			s12 = min(s12 + sp1, p12);
		if (s12 > p12)
			s12 = max(s12 - sp1, p12);
		// Back Left Lift Servo
		if (s22 < p22)
			s22 = min(s22 + sp2, p22);
		if (s22 > p22)
			s22 = max(s22 - sp2, p22);
		// Back Right Lift Servo
		if (s32 < p32)
			s32 = min(s32 + sp3, p32);
		if (s32 > p32)
			s32 = max(s32 - sp3, p32);
		// Front Right Lift Servo
		if (s42 < p42)
			s42 = min(s42 + sp4, p42);
		if (s42 > p42)
			s42 = max(s42 - sp4, p42);
		// Write servo values
		set_servos();
		// Delay before next movement
		for (uint8_t i = 0; i < spd; i++)
			_delay_ms(1);
	}
}

// Walk forward one step
void forward(void) {
	for (uint8_t i = 0; i < sizeof(srv_forward); i += 12)
		srv_move(&srv_forward[i]);
}

// Walk backward one step
void back(void) {
	for (uint8_t i = 0; i < sizeof(srv_back); i += 12)
		srv_move(&srv_back[i]);
}

// Turn left one step
void turn_left(void) {
	for (uint8_t i = 0; i < sizeof(srv_left); i += 12)
		srv_move(&srv_left[i]);
}

// Turn right one step
void turn_right(void) {
	for (uint8_t i = 0; i < sizeof(srv_right); i += 12)
		srv_move(&srv_right[i]);
}

// Move servos to center position
void center_servos(void) {
	srv_move(srv_center);
}

// Make robot bow
void bow(void) {
	center_servos();
	for (uint8_t i = 0; i < sizeof(srv_bow); i += 12)
		srv_move(&srv_bow[i]);	
}

// Make robot lean left
static void lean_left(void) {
	srv_move(srv_lean_left);
}

// Make robot lean right
static void lean_right(void) {
	srv_move(srv_lean_right);
}

// Shortest distance between two angles
static uint16_t angular_distance(uint16_t a, uint16_t b) {
	uint16_t d = (a - b + 360) % 360;
	return d > 180 ? 360 - d : d;
}

// Autonomous gait algorithm
void collision_avoid(void) {
	uint16_t distance, heading, origin = 0;
	uint8_t step = 0, adc_l, adc_r;
	state_t state = FORWARD, next_state = FORWARD;

	while (!uart_available()) {
		if (irrecv_decode(&results)) {
			irrecv_resume(); //next value
			dump_value_P("irrecv_decode=", results.value, NEWLINE);
			if (results.value != 0 && results.value != REPEAT)
				break;
		}
		// Read sensors
		distance = ping_cm();
		heading = magneto_heading();
		adc_l = read_adc(0);
		adc_r = read_adc(1);
		// Dump values
		dump_value_P("ping_cm=", distance, COMMA);
		dump_value_P("heading=", heading, COMMA);
		dump_value_P("adc_l=", adc_l, COMMA);
		dump_value_P("adc_r=", adc_r, COMMA);
		dump_value_P("state=", state, COMMA);
		dump_value_P("origin=", origin, COMMA);
		dump_value_P("next_st=", next_state, NEWLINE);
		// State machine
		if (distance < 15)
			next_state = BACKWARD;
		if ((distance >= 15 && distance < 30 && state == FORWARD) || (distance >= 30 && state == BACKWARD)) {
			origin = heading;
			next_state = adc_l > adc_r ? TURN_RIGHT : TURN_LEFT;
		}
		if (angular_distance(heading, origin) > 75 && (state == TURN_RIGHT || state == TURN_LEFT))
			next_state = FORWARD;
		// Servo position steps for gait
		switch (state) {
			case FORWARD:
				srv_move(&srv_forward[step]);
				step += 12;
				if (step >= sizeof(srv_forward)) {
					step = 0;
					state = next_state;
				}
				break;
			case TURN_RIGHT:
				srv_move(&srv_right[step]);
				step += 12;
				if (step >= sizeof(srv_right)) {
					step = 0;
					state = next_state;
				}
				break;
			case BACKWARD:
				srv_move(&srv_back[step]);
				step += 12;
				if (step >= sizeof(srv_back)) {
					step = 0;
					state = next_state;
				}
				break;
			case TURN_LEFT:
				srv_move(&srv_left[step]);
				step += 12;
				if (step >= sizeof(srv_left)) {
					step = 0;
					state = next_state;
				}
		}
	}
}

// Update how high the robot is standing
void constrain_high(void) {
	high = constrain(high, -30, 30);
	dump_value_P("high=", high, NEWLINE);
	s12 = s22 = s32 = s42 = 90 + high;
	set_servos();
}

// Update leg calibration
void constrain_trim(void) {
	trim = constrain(trim, 0, 30);
	dump_value_P("trim=", trim, NEWLINE);
	center_servos();
}

// Update walking speed
void constrain_spd(void) {
	spd = constrain(spd, 3, 50);
	dump_value_P("spd=", spd, NEWLINE);
	bow();
}

// Dump sensor status to UART
void sensor_status(void) {
	dump_value_P("ping_cm=", ping_cm(), COMMA);
	dump_value_P("heading=", magneto_heading(), COMMA);
	dump_value_P("adc_l=", read_adc(0), COMMA);
	dump_value_P("adc_r=", read_adc(1), NEWLINE);	
}

int main(void) {
	uint32_t value;
	uint32_t lastValue = 0;

	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	srvo1_init(); //reset the servo module
	srvo1a_act(); //start servo1 cha
	srvo1b_act(); //start servo1 chb
	setup_irrecv(); //enables interrupts too
	init_sonar();
	i2c_init();
	magneto_init();
	init_adc();
	init_eeprom();
	while (1) {
		if (irrecv_decode(&results)) {
			lastValue = value = results.value == REPEAT ? lastValue : results.value;
			irrecv_resume(); //next value
			dump_value_P("irrecv_decode=", value, NEWLINE);
			switch (value) {
				case 16718055: // up
					forward();
					sensor_status();
					break;
				case 16730805: // down
					back();
					sensor_status();
					break;
				case 16716015: // left
					turn_left();
					sensor_status();
					break;
				case 16734885: // right
					turn_right();
					sensor_status();
					break;
				case 16726215: // ok
					collision_avoid();
					break;
				case 16738455: // star
					lean_left();
					break;
				case 16756815: // pound
					lean_right();
					break;
				case 16750695: // 0
					center_servos();
					break;
				case 16753245: // 1
					high += 3;
					constrain_high();
					break;
				case 16736925: // 2
					trim++;
					constrain_trim();
					break;
				case 16769565: // 3
					spd++;
					constrain_spd();
					break;
				case 16720605: // 4
					high -= 3;
					constrain_high();
					break;
				case 16712445: // 5
					trim--;
					constrain_trim();
					break;
				case 16761405: // 6
					spd--;
					constrain_spd();
					break;
				case 16769055: // 7
					reset_bounds();
					break;
				case 16754775: // 8
					bow();
					break;
				case 16748655: // 9
					break;
			}
		}

		if (uart_available()) {
			switch (uart_getc()) {
				case 'w':
					forward();
					sensor_status();
					break;
				case 's':
					back();
					sensor_status();
					break;
				case 'a':
					turn_left();
					sensor_status();
					break;
				case 'd':
					turn_right();
					sensor_status();
					break;
				case 'l':
					lean_left();
					break;
				case 'r':
					lean_right();
					break;
				case 'c':
					center_servos();
					break;
				case 'b':
					bow();
					break;
				case 'u':
					high += 3;
					constrain_high();
					break;
				case 'n':
					high -= 3;
					constrain_high();
					break;
				case 't':
					trim++;
					constrain_trim();
					break;
				case 'i':
					trim--;
					constrain_trim();
					break;
				case 'e':
					reset_bounds();
					break;
				case 'f':
					spd--;
					constrain_spd();
					break;
				case 'o':
					spd++;
					constrain_spd();
					break;
				case ' ':
					collision_avoid();
					break;
				default:
					sensor_status();
			}
		}
	}
}
