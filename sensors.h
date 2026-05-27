//Team 10
//Engr290

#ifndef SENSORS_H
#define SENSORS_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//Sensor readings
extern float left_dist;
extern float right_dist;
extern float front_dist;
extern float yaw_deg;
extern float gz_rate;

//Raw ADC values for IR sensors
extern uint16_t left_adc_raw;
extern uint16_t right_adc_raw;

void sensors_init(void);
void update_ir(void);
void update_us(void);
void update_imu(float dt);

#ifdef __cplusplus
}
#endif
#endif