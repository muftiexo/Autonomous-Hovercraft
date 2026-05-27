//Team 10
//Engr290

#include "controlSys.h"
#include "decision.h"
#include "sensors.h"
#include "hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

//Use of PD Control system
#define KP_YAW  2.2f
#define KD_YAW  4.0f

//KP_turn should be between 1.4 adn 1.5 for better turns
#define KP_TURN 1.45f

#define TURN_MIN_SERVO 60.0f

//speed adjustments at different scenarios
#define THRUST_STRAIGHT 255
#define THRUST_BRAKE 76
#define THRUST_TURN 220
#define LIFT_NORMAL 255
#define LIFT_BRAKE 120

static float prev_error = 0.0f;

static float pd_control(float target, float current){
    float err = angle_diff(target, current);
    float deriv = err - prev_error;
    float out = -(KP_YAW*err) -(KD_YAW* deriv);
    prev_error = err;
    if(out > 80.0f){out = 80.0f;}
    if(out < -80.0f){out = -80.0f;}
    return out;
}

static float turn_control(float target, float current){
    float err = angle_diff(target, current);
    if(fabs(err)<2.0f)
        return 0.0f;

    float out = -(KP_TURN * err);

    if(out> 0.0f && out<TURN_MIN_SERVO){
        out = TURN_MIN_SERVO;
        }
    if(out<0.0f && out> -TURN_MIN_SERVO){
        out = -TURN_MIN_SERVO;
        }

    if(out>80.0f){out = 80.0f;}
    if(out<-80.0f){out = -80.0f;}
    return out;
}

void apply_control(){
    static NavState prev = ST_STRAIGHT;
    
    //actual speed currently being sent to the hardware
    static float current_thrust = 0.0f; 
    
    float servo_cmd = 0.0f;
    uint8_t target_thrust = THRUST_STRAIGHT;
    uint8_t lift_cmd = LIFT_NORMAL;

    if(nav_state != prev){
        prev_error = 0.0f;
        prev = nav_state;
    }

    switch(nav_state){
        case ST_STRAIGHT:
            servo_cmd = pd_control(yaw_target, yaw_deg);
            target_thrust = THRUST_STRAIGHT;
            lift_cmd = LIFT_NORMAL;
            break;

        case ST_BRAKE:
            servo_cmd = pd_control(yaw_target, yaw_deg);
            target_thrust = THRUST_BRAKE;
            lift_cmd = LIFT_BRAKE;
            break;

        case ST_TURN_1:
            servo_cmd = turn_control(yaw_target, yaw_deg);
            target_thrust = THRUST_TURN;
            lift_cmd = LIFT_NORMAL;
            break;

        case ST_TURN_2:
            servo_cmd = turn_control(yaw_target, yaw_deg);
            target_thrust = THRUST_TURN;
            lift_cmd = LIFT_NORMAL;
            break;

        case ST_CROSS:
            servo_cmd = pd_control(yaw_target, yaw_deg);
            target_thrust = THRUST_STRAIGHT;
            lift_cmd = LIFT_NORMAL;
            break;
    }

    if(current_thrust < target_thrust){
        if(nav_state == ST_TURN_1 || nav_state == ST_TURN_2){
            current_thrust += 10.0f;
        } else {
            current_thrust += 2.0f;
        }
        if(current_thrust > target_thrust){
            current_thrust = target_thrust;
        }
    }
    else if(current_thrust > target_thrust){
        current_thrust = target_thrust; 
    }

    setServo(servo_cmd);
    setSpeedThrust((uint8_t)current_thrust);
    setSpeedLift(lift_cmd);
}