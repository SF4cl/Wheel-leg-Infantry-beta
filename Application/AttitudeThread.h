#ifndef __AttitudeThread_H
#define __AttitudeThread_H
#include "struct_typedef.h"
#include "MahonyAHRS.h"
#include "YC_ICM42688.h"
#include "cmsis_os.h"
#include "tim.h"
#include "stm32h7xx_it.h"
#include "Control.h"
#include "fdcan.h"
#include "kalman filter.h"
#include "QuaternionEKF.h"
#include "user_lib.h"
extern void AttitudeThread(void *argument);
extern void get_angle(fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll);
extern first_order_filter_type_t accel_filter;

extern fp32 INS_quat[4];
extern fp32 INS_angle[3];
extern fp32 INS_accel[3];
extern fp32 Eaccel[3],Baccel[3];
#endif
