#include "chassis.h"
#include "speed_control.h"
void Chassis_SetLeftRight(float left, float right)
{
    float targets[MOTOR_COUNT] = {0};
    targets[LEFT_FRONT_MOTOR]=left;
    targets[LEFT_REAR_MOTOR]=left;
    targets[RIGHT_FRONT_MOTOR]=right;
    targets[RIGHT_REAR_MOTOR]=right;
    SpeedControl_SetTargets(targets);
}
void Chassis_SetForwardTurn(float forward_mm_s, float right_turn_delta_mm_s)
{
    Chassis_SetLeftRight(forward_mm_s+right_turn_delta_mm_s,forward_mm_s-right_turn_delta_mm_s);
}
