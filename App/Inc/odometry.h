#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "encoder.h"
typedef struct { float left_mm, right_mm, center_mm; } OdomData;
void Odometry_FromEncoder(const EncoderData *encoder, OdomData *odom);
#endif
