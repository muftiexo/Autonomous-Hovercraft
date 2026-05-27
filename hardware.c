//Team 10
//Engr290

//Sometimes gets stuck for left turns maybe because of spikes in voltage or equal distance for IR sensors

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>

#include "hardware.h"

//DT CALCULATION FOR BETTER IMU READING
volatile uint32_t timer_overflow = 0;

ISR(TIMER2_OVF_vect){ 
    timer_overflow++; 
}

void dtTimerBegin(void){
    //clears timer/counter registers
    TCNT2 = 0;
    TCCR2A = 0;
    TCCR2B = 0;
    //clears software counter
    timer_overflow = 0;
    //clears timer interrupt flag
    TIFR2 = (1 << TOV2);
    //allows ISR
    TIMSK2 = (1 << TOIE2);
    //1/(16MHZ/1024) = 1 tick per 64us
    TCCR2B = (1 << CS22)|(1 << CS21)|(1 << CS20);
}

float dtRead(void){
    uint8_t status_register = SREG;
    //clears interrupts
    cli();
    uint32_t overflow = timer_overflow;
    uint8_t count = TCNT2;
    timer_overflow = 0;
    TCNT2 = 0;
    SREG = status_register;
    //1 overfow count = 256 ticks. 
    //adds the remain current number of ticks
    //multiply by 64us to convert in seconds
    float dt =(overflow * 256 + count) * 0.000064f;
    //deadband to ensure accurate dt measurement
    if(dt < 0.001f || dt > 0.5f){
        return 0.0f;
        }
    return dt;
}

// Servo on PB1/OC1A — Timer1 Fast PWM, ICR1 = 40000, prescaler 8
void servo_init(void){
    //output = PB1
    DDRB |=(1 << PB1);
    //WGM = 1110 - Fast PWN mode
    TCCR1A =(1 << COM1A1) | (1 << WGM11);
    //clock : 1/(16MHZ/8) = 0.5 seconds per tick
    TCCR1B =(1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 40000;
    OCR1A = 3000; //center or 0 deg
}

void setServo(float angle_deg){
    if(angle_deg > 85.0f){angle_deg = 85.0f;}
    if(angle_deg < -85.0f){angle_deg = -85.0f;}
    //-85deg = 2000, 0deg = 3000, +85deg = 4000
    OCR1A =(uint16_t)(2000+((angle_deg + 85.0f)* 2000.0f)/ 170.0f);
}

//Fans - Timer0 Fast PWM
void fans_init(void){
    //outputs = PD5 and PD6
    DDRD |=(1 << PD5) | (1 << PD6);
    TCCR0A =(1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B =(1 << CS01) | (1 << CS00);
    OCR0A = 0; // lift off
    OCR0B = 0; // thrust off
}

void setSpeedLift(uint8_t rpm){
    OCR0A = rpm;
}

void setSpeedThrust(uint8_t rpm){
    OCR0B = rpm;
}

// UART at 9600
#define BAUD 9600UL
#define UBRR_VAL ((F_CPU /(16UL * BAUD)) - 1)

ISR(USART_TX_vect){}

void uart_init(void){
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)UBRR_VAL;
    UCSR0B = (1 << TXEN0) | (1 << TXCIE0);
    UCSR0C = (3 << UCSZ00);
}

void uart_print(const char *str){
    while(*str){
        while(!(UCSR0A & (1 << UDRE0)));
        UDR0 = *str++;
    }
}

void uart_print_int(int32_t num){
    char buf[12];
    ltoa(num, buf, 10);
    uart_print(buf);
}

void uart_print_float(float val){
    if(val < 0){
        uart_print("-"); val = -val;
    }
    int32_t i = (int32_t)val;
    int32_t d = (int32_t)((val - i)* 100);
    uart_print_int(i);
    uart_print(".");
    if(d < 10) uart_print("0");
    uart_print_int(d);
}

//Overall init
void hardware_init(){
    cli();
    // GPIO defaults
    DDRB |=(1 << PB3); // US trigger
    DDRD &= ~(1 << PD2); // US echo input
    PORTD |=(1 << PD2);
    servo_init();
    fans_init();
    uart_init();

    // ADC: external AREF, prescaler 128
    ADMUX = 0x00;
    ADCSRA =(1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    sei();
}