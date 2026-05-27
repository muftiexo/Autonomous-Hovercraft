//Team 10
//Engr290

#ifndef HARDWARE_H
#define HARDWARE_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void hardware_init();

// Servo: takes angle in degrees (-85 to +85)
void setServo(float angle_deg);

void setSpeedLift(uint8_t rpm);
void setSpeedThrust(uint8_t rpm);

// UART
void uart_init(void);
void uart_print(const char *str);
void uart_print_int(int32_t num);
void uart_print_float(float val);

// Timing
void dtTimerBegin(void);
float dtRead(void);

#ifdef __cplusplus
}
#endif
#endif