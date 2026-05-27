//Team 10
//Engr290

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#include "sensors.h"

//measurements variables
float left_dist = 40.0f;
float right_dist = 40.0f;
float front_dist = 100.0f;
float yaw_deg = 0.0f;
float gz_rate = 0.0f;

uint16_t left_adc_raw  = 0;
uint16_t right_adc_raw = 0;
uint16_t us_raw_count  = 0;

// IMU
#define IMU_ADDR 0x68
#define GYRO_CONFIG  0x1B
#define FS_SEL        0x08

static int16_t gz_off = 0;

static void i2c_start(void){
    uint16_t timeout = 0;
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while(!(TWCR & (1<<TWINT))) {
        if(++timeout > 10000) return;
    }
}

static void i2c_stop(void){
    uint16_t timeout = 0;
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
    while(TWCR & (1<<TWSTO)) {
        if(++timeout > 10000) return;
    }
}

static void i2c_write(uint8_t d){
    uint16_t timeout = 0;
    TWDR = d;
    TWCR =(1<<TWINT)|(1<<TWEN);
    while(!(TWCR & (1<<TWINT))) {
        if(++timeout > 10000) return;
    }
}

static uint8_t i2c_read_ack(void){
    uint16_t timeout = 0;
    TWCR =(1<<TWINT)|(1<<TWEN)|(1<<TWEA);
    while(!(TWCR & (1<<TWINT))) {
        if(++timeout > 10000) return 0;
    }
    return TWDR;
}

static uint8_t i2c_read_nack(void){
    uint16_t timeout = 0;
    TWCR =(1<<TWINT)|(1<<TWEN);
    while(!(TWCR & (1<<TWINT))) {
        if(++timeout > 10000) return 0;
    }
    return TWDR;
}

static void i2c_write_reg(uint8_t reg, uint8_t val){
    i2c_start();
    i2c_write(IMU_ADDR << 1);
    i2c_write(reg);
    i2c_write(val);
    i2c_stop();
}

void sensors_init(void){
    // I2C: 100kHz
    TWSR = 0;
    TWBR = 72;

    i2c_write_reg(0x6B, 0x01);
    _delay_ms(100);
    i2c_write_reg(0x19, 0x07);
    i2c_write_reg(0x1A, 0x03);
    i2c_write_reg(GYRO_CONFIG, FS_SEL);

    int32_t sum = 0;
    for(uint8_t i = 0; i < 100; i++) {
        i2c_start();
        i2c_write(IMU_ADDR << 1);
        i2c_write(0x47);
        i2c_start();
        i2c_write((IMU_ADDR << 1) | 1);
        int16_t raw =(i2c_read_ack() << 8) | i2c_read_nack();
        i2c_stop();
        sum += raw;
        _delay_ms(5);
    }
    gz_off = sum / 100;
}

void update_imu(float dt){
    i2c_start();
    i2c_write(IMU_ADDR << 1);
    i2c_write(0x47);
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);
    int16_t raw =(i2c_read_ack() << 8) | i2c_read_nack();
    i2c_stop();

    gz_rate =(raw - gz_off) / 65.5f;  // ±500°/s LSB:65.5
    if(gz_rate > -1.5f && gz_rate < 1.5f) gz_rate = 0.0f;
    yaw_deg += gz_rate * dt;
    if(yaw_deg > 180.0f) yaw_deg -= 360.0f;
    if(yaw_deg < -180.0f) yaw_deg += 360.0f;
}

// ADC
static uint16_t adc_read(uint8_t ch){
    ADMUX =(ADMUX & 0xF0) | (ch & 0x07);
    _delay_us(10);
    ADCSRA |= (1 << ADSC);
    while(ADCSRA &(1 << ADSC));
    return ADC;  // full 10-bit
}

//calibration tables
static float adc_to_cm_left(uint16_t adc){
    if(adc > 690) return 7.0f;
    if(adc > 654) return 8.0f;
    if(adc > 604) return 9.0f;
    if(adc > 564) return 10.0f;
    if(adc > 484) return 12.0f;
    if(adc > 426) return 14.0f;
    if(adc > 380) return 16.0f;
    if(adc > 349) return 18.0f;
    if(adc > 321) return 20.0f;
    if(adc > 298) return 22.0f;
    if(adc > 281) return 24.0f;
    if(adc > 266) return 26.0f;
    if(adc > 243) return 28.0f;
    if(adc > 233) return 30.0f;
    if(adc > 211) return 35.0f;
    if(adc > 191) return 40.0f;
    if(adc > 176) return 45.0f;
    if(adc > 168) return 50.0f;
    if(adc > 157) return 55.0f;
    if(adc > 146) return 60.0f;
    return 65.0f;
}

static float adc_to_cm_right(uint16_t adc){
    if(adc > 693) return 7.0f;
    if(adc > 631) return 8.0f;
    if(adc > 584) return 9.0f;
    if(adc > 543) return 10.0f;
    if(adc > 465) return 12.0f;
    if(adc > 407) return 14.0f;
    if(adc > 363) return 16.0f;
    if(adc > 332) return 18.0f;
    if(adc > 301) return 20.0f;
    if(adc > 280) return 22.0f;
    if(adc > 260) return 24.0f;
    if(adc > 242) return 26.0f;
    if(adc > 230) return 28.0f;
    if(adc > 217) return 30.0f;
    if(adc > 190) return 35.0f;
    if(adc > 166) return 40.0f;
    if(adc > 157) return 45.0f;
    if(adc > 147) return 50.0f;
    if(adc > 138) return 55.0f;
    if(adc > 130) return 60.0f;
    return 65.0f;
}

void update_ir(void){
    uint32_t sum_l = 0, sum_r = 0;
    adc_read(0);
    for(uint8_t i = 0; i < 8; i++) {
        sum_l += adc_read(0);  // left on ADC0
    }
    adc_read(1);
    for(uint8_t i = 0; i < 8; i++) {
        sum_r += adc_read(1);  // right on ADC1
    }
    left_adc_raw =(uint16_t)(sum_l / 8);
    right_adc_raw =(uint16_t)(sum_r / 8);
    left_dist  = adc_to_cm_left(left_adc_raw);
    right_dist = adc_to_cm_right(right_adc_raw);
}

void update_us(void){
    uint16_t timeout;
    PORTB |= (1 << PB3);
    _delay_us(10);
    PORTB &= ~(1 << PB3);
    timeout = 0;
    while(!(PIND &(1 << PD2))){
        _delay_us(1);
        if(++timeout > 30000){
            front_dist = 999.0f; return;
            }
    }

    uint16_t us_count = 0;
    while(PIND & (1 << PD2)){
        _delay_us(1);
        if(++us_count > 25000){
            front_dist = 999.0f; return;
        }
    }

    float new_cm =(float)us_count / 155.0f;

    // Simple low-pass filter
    front_dist = 0.5f * front_dist + 0.5f * new_cm;
}