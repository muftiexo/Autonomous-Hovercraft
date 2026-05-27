//Team 10
//Engr290

/*
 PORT MAPPING:
 servo : P9 - PB1(OC1A) - Timer1 PWM
 lift fan : P4 - PD6(OC0A) - Timer0 PWM
 thrust fan : P3 - PD5(OC0B) - Timer0 PWM
 US : P6 - PB3/PD2 - GPIO output - GPIO input
 left IR : P5 - PC0 (ADC0) - Analog input
 right IR : P8 - PC1 (ADC1) - Analog input
 IMU : P19 - PC4/PC5 - I2C
 */

/*
Timer0 = fans
Timer1 = servo
Timer2 = dt
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#include "hardware.h"
#include "sensors.h"
#include "decision.h"
#include "controlSys.h"

int main(void){
    hardware_init();
    _delay_ms(200);

    uart_print("Init of sensors...\r\n");
    sensors_init();

    uart_print("Calibrated\r\n");
    decision_init();

    //Minize hovercraft drifting at the beginning 
    setSpeedLift(180);
    _delay_ms(2000);

    yaw_deg = 0.0f;
    yaw_target = 0.0f;
    setServo(0.0f);

    dtTimerBegin();

    //set lift fan to full speed
    setSpeedLift(255);

    uint8_t sensor_count = 0;
    uint8_t print_count = 0;

    while (1) {
        // IMU every loop with real dt
        float dt = dtRead();
        update_imu(dt);

        sensor_count++;
        if(sensor_count >= 4){
            update_ir();
            update_us();
            sensor_count = 0;
        }

        update_decision();
        apply_control();

        print_count++;
        if(print_count >= 20){
            uart_print("S:");
            uart_print_int(nav_state);
            uart_print(" Y:");
            uart_print_float(yaw_deg);
            uart_print(" T:");
            uart_print_float(yaw_target);
            uart_print(" US:");
            uart_print_float(front_dist);
            uart_print(" L:");
            uart_print_float(left_dist);
            uart_print(" R:");
            uart_print_float(right_dist);
            uart_print(" LADC:");
            uart_print_int(left_adc_raw);
            uart_print(" RADC:");
            uart_print_int(right_adc_raw);
            uart_print("\r\n");
            print_count = 0;
        }

        _delay_ms(20);
    }

    return 0;
}