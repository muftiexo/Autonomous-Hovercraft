//Team 10
//Engr290


#ifndef DECISION_H
#define DECISION_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum{
    ST_STRAIGHT,
    ST_BRAKE,
    ST_TURN_1,
    ST_CROSS,
    ST_TURN_2
} NavState;

typedef enum{
    DIR_LEFT,
    DIR_RIGHT,
    DIR_NONE
} TurnDir;

extern NavState nav_state;
extern TurnDir turn_dir;
extern float yaw_target;

void decision_init(void);
void update_decision();
float angle_diff(float a, float b);

#ifdef __cplusplus
}
#endif
#endif