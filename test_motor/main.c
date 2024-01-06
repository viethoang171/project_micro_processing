/*
 * test_motor.c
 *
 * Created: 12/21/2023 9:20:35 PM
 * Author : DELL
 */

#include <avr/io.h>
#define F_CPU 16000000UL

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <avr/delay.h>
#include <avr/interrupt.h>

#include "i2c_lcd.h"
#include "i2c.h"

#define IN1 5
#define IN2 7
#define EN 6

#define PWM_PERIOD 16000
#define SAMPLING_TIME 15	 // ms
#define INV_SAMPLING_TIME 67 // Hz

volatile int16_t i16_pulse = 0;
volatile int16_t i16_pre_Pulse = 0;
volatile int16_t rSpeed, Err, pre_Err;	  // for speed control
volatile uint8_t Kp = 8, Kd = 10, Ki = 1; // PID coefficient
volatile uint8_t Ctrl_speed = 5;		  // desired speed
volatile int16_t out_put_pulse;

volatile uint8_t flag = 0;

uint8_t sample_counter = 0;

char *text = NULL;

// Function convert int to string
char *intToString(int16_t num)
{
	int temp = num;
	int digitCount = 0;

	do
	{
		temp /= 10;
		digitCount++;
	} while (temp != 0);

	// Define signed number
	int sign = (num < 0) ? -1 : 1;

	// Allocate memory
	char *result = (char *)malloc(digitCount + 2); // 1 cho mỗi chữ số và 1 cho dấu

	// Process signed number
	if (sign == -1)
	{
		result[0] = '-';
	}

	// Convert integer to string
	for (int i = digitCount - 1; i >= 0; i--)
	{
		result[sign == -1 ? i + 1 : i] = abs(num % 10) + '0';
		num /= 10;
	}

	// End string
	result[digitCount + (sign == -1 ? 1 : 0)] = '\0';

	return result;
}

void motor_speed_PID(int32_t desired_speed)
{

	rSpeed = i16_pulse - i16_pre_Pulse; // caculate speed
	i16_pre_Pulse = i16_pulse;			// save Pulse value
	Err = desired_speed - abs(rSpeed);	// caculate error

	// PID's components
	int16_t pPart = Kp * Err;
	int16_t dPart = Kd * (Err - pre_Err) * INV_SAMPLING_TIME;
	int16_t iPart = Ki * SAMPLING_TIME * Err / 1000;
	out_put_pulse += pPart + iPart + dPart;

	// Reconfigure when output get saturation
	if (out_put_pulse >= PWM_PERIOD)
	{
		out_put_pulse = PWM_PERIOD - 1;
	}
	if (out_put_pulse <= 0)
	{
		out_put_pulse = 1;
	}
	OCR0A = out_put_pulse; // get duty for OCR0A: update PWM
	pre_Err = Err;		   // save Error value
}

LiquidCrystalDevice_t device;
int main(void)
{
	// Motor
	DDRD = (1 << EN) | (1 << IN1) | (1 << IN2);
	DDRB = 0x00;

	PORTD = (1 << 2) | (1 << 3);
	EICRA = 0x0A; // make INT0 and INT1 falling edge triggered

	// Use Timer2 for timer 25ms, sampling time
	TCCR2B = (1 << CS02) | (1 << CS00); // CS02=1, CS01=0, CS00=1: Prescaler 1024
	TCNT2 = 21;							// set init value for T/C2 to get 15 ms (f=16MHz)
	TIMSK2 = (1 << TOIE2);				// alow interrupt when over flow T/C0

	// Use Timer0 for generate PWM
	TCCR0A |= (1 << WGM01) | (1 << WGM00) | (1 << COM0A1);
	TCCR0B |= (1 << CS02) | (1 << CS00);

	EIMSK = (1 << INT0) | (1 << INT1); // enable external interrupt 0
	sei();							   // enable interrupts

	// I2C LCD
	i2c_master_init(I2C_SCL_FREQUENCY_100);

	device = lq_init(0x27, 16, 2, 0);
	lq_turnOnBacklight(&device);

	while (1)
	{
	};
}

ISR(INT0_vect) // ISR for external interrupt 0
{
	if (bit_is_set(PINB, 0) != 0)
	{
		i16_pulse++;
	}
	else
	{
		i16_pulse--;
	}
}

ISR(INT1_vect) // ISR for external interrupt 1
{

	flag++;
	if (flag % 2 != 0)
	{
		PORTD |= (1 << IN1);
		motor_speed_PID(100);

		lq_clear(&device);
		lq_setCursor(&device, 1, 0);
		lq_print(&device, "Run_DC");

		if (bit_is_set(PINB, 0) != 0)
		{
			lq_setCursor(&device, 0, 0);
			lq_print(&device, "quay_nghich");
		}
		else
		{
			lq_setCursor(&device, 0, 0);
			lq_print(&device, "quay_thuan");
		}
	}
	else
	{
		PORTD &= ~(1 << IN1);
		text = "Pause_DC";
		lq_clear(&device);
		lq_setCursor(&device, 0, 0);
		lq_print(&device, text);
	}
}

ISR(TIMER2_OVF_vect) // update sampling time
{
	sample_counter++;
	if (bit_is_set(PINB, 1) == 0)
	{
		lq_setCursor(&device, 1, 0);
		Ctrl_speed = 100;
		lq_print(&device, "PB1_is_pressed");
	}

	if (sample_counter == 80) // sample every 1200ms
	{
		if (flag % 2 == 1)
		{
			lq_clear(&device);
			lq_setCursor(&device, 0, 0);
			lq_print(&device, "act_speed:");
			lq_setCursor(&device, 0, 11);
			text = intToString(out_put_pulse);
			lq_print(&device, text);
			if (bit_is_set(PINB, 0) != 0)
			{
				lq_setCursor(&device, 1, 0);
				lq_print(&device, "quay nghich");
			}
			else
			{
				lq_setCursor(&device, 1, 0);
				lq_print(&device, "quay thuan");
			}
		}

		// reset sample counter
		sample_counter = 0;
	}

	TCNT2 = 21; // set init value for T/C2
	if (flag % 2 != 0)
		motor_speed_PID(200);
}