/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef MCL_APP_CONFIG_H
#define MCL_APP_CONFIG_H

/* 电机控制库功能配置 */
#define MCL_EN_THETA_FORECAST (1)              /* 电角度预测功能使能 */
#define MCL_EN_DQ_AXIS_DECOUPLING_FUNCTION (1) /* DQ轴解耦功能使能 */
#define MCL_EN_DEAD_AREA_COMPENSATION (1)      /* 死区补偿功能使能 */
#define MCL_EN_SENSORLESS_SMC (0)             /* 无感滑模控制功能使能 */

#endif
