#include "odometry.h"
void Odometry_FromEncoder(const EncoderData *encoder, OdomData *odom)
{
    const float mm_per_count=3.14159265359f*WHEEL_DIAMETER_MM/WHEEL_COUNTS_PER_REV;
    odom->left_mm=((float)encoder->count[LEFT_FRONT_MOTOR]+(float)encoder->count[LEFT_REAR_MOTOR])*0.5f*mm_per_count;
    odom->right_mm=((float)encoder->count[RIGHT_FRONT_MOTOR]+(float)encoder->count[RIGHT_REAR_MOTOR])*0.5f*mm_per_count;
    odom->center_mm=(odom->left_mm+odom->right_mm)*0.5f;
}
