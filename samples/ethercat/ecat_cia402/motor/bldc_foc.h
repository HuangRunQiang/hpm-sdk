/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef BLDC_FOC_H
#define BLDC_FOC_H

#include "hpm_mcl_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* 电机功能初始化 */
    void motor_function_init(void);

    /* 电机速度环初始化 */
    void motor_speed_loop_init(void);

    /* 设置电机目标速度 */
    void motor_speed_loop_set(int32_t target_speed);

    /* 电机位置环初始化 */
    void motor_postion_loop_init(void);

    /* 设置电机目标位置 */
    void motor_position_loop_set(int32_t target_position);

    /* 获取电机实际速度 */
    int32_t motor_get_actual_speed(void);

    /* 获取电机实际位置 */
    int32_t motor_get_actual_position(void);

    /* 使能电机 */
    void motor_enable(void);

    /* 失能电机 */
    void motor_disable(void);

#ifdef __cplusplus
}
#endif

#endif /* BLDC_FOC_H */