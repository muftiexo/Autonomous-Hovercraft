//Team 10
//Engr290

#include "decision.h"
#include "sensors.h"
#include "hardware.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

//Global variables
NavState nav_state = ST_STRAIGHT;
TurnDir turn_dir = DIR_NONE;
float yaw_target = 0.0f;

static float final_yaw = 0.0f;
static uint8_t brake_count = 0;
static uint8_t cross_count = 0;
static uint8_t confirm_count = 0;

static uint16_t wall_stuck_counter = 0;
static uint16_t turn_timeout = 0;

//Thresholds 
#define FRONT_WALL_DETECT_CM 65.0f
#define SIDE_OPEN_CM 40.0f

#define BRAKE_MIN_LOOPS 40
#define BRAKE_MAX_LOOPS 55\

#define CROSS_MIN_LOOPS 8
#define CROSS_MAX_LOOPS 20

#define TURN_DONE_DEG 12.0f
#define CONFIRM_NEEDED 3

#define STUCK_THRESHOLD   15.0f
#define STUCK_TIMEOUT     250

#define TURN_MAX_LOOPS    250
#define CROSS_WALL_CM     25.0f

float angle_diff(float a, float b){
    float d = a - b;
    if(d>180.0f){
        d -= 360.0f;
        }
    if(d< -180.0f){
        d += 360.0f;
        }
    return d;
}

static float clamp_angle(float angle){
    if(angle>180.0f){
        angle -= 360.0f;
        }
    if(angle< -180.0f){
        angle += 360.0f;
        }
    return angle;
}

static TurnDir pick_direction(void){
    uint8_t lo = (left_dist >= SIDE_OPEN_CM);
    uint8_t ro = (right_dist >= SIDE_OPEN_CM);
    if(lo && !ro)return DIR_LEFT;
    if(ro && !lo) return DIR_RIGHT;
    return DIR_RIGHT;
}

void decision_init(void){
    nav_state = ST_STRAIGHT;
    turn_dir = DIR_NONE;
    yaw_target = 0.0f;
    final_yaw = 0.0f;
    brake_count = 0;
    cross_count = 0;
    confirm_count = 0;
}

void update_decision(){
    //STUCK DETECTION
    if(front_dist < STUCK_THRESHOLD && (nav_state == ST_STRAIGHT || nav_state == ST_BRAKE)){
        wall_stuck_counter++;
        if(wall_stuck_counter >= 100){
            setSpeedThrust(0);
            setSpeedLift(120);
        }

        if(wall_stuck_counter >= STUCK_TIMEOUT){
            yaw_target = yaw_deg;
            nav_state = ST_STRAIGHT;
            confirm_count = 0;
            brake_count = 0;
            wall_stuck_counter = 0;
        }
    } else {
        wall_stuck_counter = 0;
    }

    switch(nav_state){
        case ST_STRAIGHT:{
            if(front_dist <= FRONT_WALL_DETECT_CM){
                confirm_count++;
                if(confirm_count >= CONFIRM_NEEDED){
                    brake_count   = 0;
                    confirm_count = 0;
                    nav_state     = ST_BRAKE;
                }
            }else{
                confirm_count = 0;
            }
            break;
        }

        case ST_BRAKE:{
            brake_count++;

            if(brake_count >= BRAKE_MIN_LOOPS && (front_dist <= FRONT_WALL_DETECT_CM || brake_count >= BRAKE_MAX_LOOPS)){

                turn_dir = pick_direction();

                float turn_angle =(turn_dir == DIR_LEFT) ? 90.0f : -90.0f;
                yaw_target = clamp_angle(yaw_target + turn_angle);
                final_yaw  = clamp_angle(yaw_target + turn_angle);
                turn_timeout = 0;
                nav_state  = ST_TURN_1;
            }
            break;
        }
        case ST_TURN_1:{
            turn_timeout++;

            if(fabs(angle_diff(yaw_target, yaw_deg))<= TURN_DONE_DEG){
                cross_count  = 0;
                turn_timeout = 0;
                nav_state    = ST_CROSS;
            }
            else if(turn_timeout >= TURN_MAX_LOOPS){
                yaw_target   = yaw_deg; 
                final_yaw    = clamp_angle(yaw_target + ((turn_dir == DIR_LEFT) ? 90.0f : -90.0f));
                cross_count  = 0;
                turn_timeout = 0;
                nav_state    = ST_CROSS;
            }
            break;
        }

        case ST_CROSS:{
            cross_count++;

            if(cross_count >= CROSS_MIN_LOOPS && (front_dist<CROSS_WALL_CM || cross_count >= CROSS_MAX_LOOPS)){
                yaw_target   = final_yaw;
                turn_timeout = 0;
                nav_state    = ST_TURN_2;
            }
            break;
        }

        case ST_TURN_2:{
            turn_timeout++;

            if(fabs(angle_diff(yaw_target, yaw_deg)) <= TURN_DONE_DEG){
                confirm_count = 0;
                turn_timeout  = 0;
                nav_state     = ST_STRAIGHT;
            }
            
            else if(turn_timeout >= TURN_MAX_LOOPS){
                yaw_target    = yaw_deg;
                confirm_count = 0;
                turn_timeout  = 0;
                nav_state     = ST_STRAIGHT;
            }
            break;
        }
    }
}