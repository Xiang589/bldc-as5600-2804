#ifndef MOTOR_CONTROL_CONFIG_H
#define MOTOR_CONTROL_CONFIG_H

/*
 * Motor constants from the 2804 drawing.
 * Keep these separate from tuned controller gains so board bring-up changes are easy to audit.
 */
#define MOTOR_FOC_POLE_PAIRS                     7
#define MOTOR_FOC_SUPPLY_MV                      12000

/*
 * Conservative voltage-mode FOC bring-up parameters.
 * Uq is a voltage command only; this project still has no verified phase-current loop.
 */
#define MOTOR_FOC_VOLTAGE_LIMIT_MV               1000
#define MOTOR_FOC_ALIGN_VOLTAGE_MV               500
#define MOTOR_FOC_ALIGN_TIME_MS                  250U
#define MOTOR_FOC_ANGLE_TIMEOUT_MS               120U
#define MOTOR_FOC_UQ_SLEW_MV                     20
#define MOTOR_FOC_ALLOW_SOFTWARE_ZERO_FALLBACK   0U

/*
 * The captured zero offset represents the rotor d-axis. Runtime Uq output must be shifted
 * by +/-90 electrical degrees. If FOCV is weak, reversed, or shakes, check sensor sign,
 * q-axis sign, and phase order before increasing voltage.
 */
#define MOTOR_FOC_Q_AXIS_OFFSET_DEG              90
#define MOTOR_FOC_Q_AXIS_SIGN                    1
#define MOTOR_FOC_SENSOR_DELTA_SIGN              (-1)

#define MOTOR_FOC_POSITION_STEP_DEG_X10          100
#define MOTOR_FOC_POSITION_LIMIT_DEG_X10         36000
#define MOTOR_FOC_POSITION_ERROR_LIMIT_DEG_X10   7200
#define MOTOR_FOC_TARGET_RPM_LIMIT_X10           1200

#define MOTOR_FOC_SPEED_PID_KP_NUM               1
#define MOTOR_FOC_SPEED_PID_KP_DEN               4
#define MOTOR_FOC_SPEED_PID_KI_NUM               1
#define MOTOR_FOC_SPEED_PID_KI_DEN               80
#define MOTOR_FOC_SPEED_PID_KD_NUM               0
#define MOTOR_FOC_SPEED_PID_KD_DEN               1
#define MOTOR_FOC_SPEED_PID_INTEGRATOR_LIMIT     6000
#define MOTOR_FOC_SPEED_PID_OUTPUT_LIMIT_MV      MOTOR_FOC_VOLTAGE_LIMIT_MV

#define MOTOR_FOC_POSITION_PID_KP_NUM            1
#define MOTOR_FOC_POSITION_PID_KP_DEN            30
#define MOTOR_FOC_POSITION_PID_KI_NUM            0
#define MOTOR_FOC_POSITION_PID_KI_DEN            1
#define MOTOR_FOC_POSITION_PID_KD_NUM            0
#define MOTOR_FOC_POSITION_PID_KD_DEN            1
#define MOTOR_FOC_POSITION_PID_INTEGRATOR_LIMIT  0

/*
 * Feedback sampling stays in FeedbackTask, not in interrupts or ControlTask.
 * These values keep I2C1 at the existing CubeMX timing while improving FOC loop visibility.
 */
#define MOTOR_FEEDBACK_TASK_PERIOD_MS            10U
#define MOTOR_FEEDBACK_ANGLE_PERIOD_MS           10U
#define MOTOR_FEEDBACK_SPEED_PERIOD_MS           50U
#define MOTOR_FEEDBACK_DIAG_PERIOD_MS            200U

#endif /* MOTOR_CONTROL_CONFIG_H */
