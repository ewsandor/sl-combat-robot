/*
  sl_cr_config.h
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  April 2022
*/
#ifndef __SL_CR_CONFIG_H__
#define __SL_CR_CONFIG_H__

/////////////////////////////////////////////////////////////////
/* This is the main configuration file for a Combat Robot.
 *    This file may be used to configure application 
 *    specific values and settings at setup, but should not be 
 *    directly referenced in any generic or reusable code 
 *    (e.g. motor drivers, drive strategy, control loops, etc).*/
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
/////////////////TOP LEVEL CONFIGURATION ////////////////////////
//#define _BENCH_SAFE_MODE_
//#define _COMBAT_MODE_
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
//////////////////// FEATURIZATION //////////////////////////////
#define _ARCADE_DRIVE_
//#define _FORCE_LIMP_MODE_
//#define _SERIAL_DEBUG_MODE_
/////////////////////////////////////////////////////////////////
//////////////// IMPLICIT FEATURIZATION /////////////////////////
#ifdef _BENCH_SAFE_MODE_
  #define SL_CR_ACTIVE_LOG_LEVEL LOG_LEVEL_ALL
  #ifndef _SERIAL_DEBUG_MODE_
  #define _SERIAL_DEBUG_MODE_
  #endif
  #define _VIRTUAL_MOTORS_
  #undef  _COMBAT_MODE_
#endif
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
//////////////////// Logging Config /////////////////////////////
/* Debug Level (see log_level_e for levels) */
#ifndef SL_CR_ACTIVE_LOG_LEVEL
//#define SL_CR_ACTIVE_LOG_LEVEL LOG_LEVEL_ALL
#define SL_CR_ACTIVE_LOG_LEVEL LOG_LEVEL_INFO
#endif
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
/////////////////// Pin Assignments /////////////////////////////
#define SL_CR_PIN_DRIVE_MOTOR_1_IN1   2
#define SL_CR_PIN_DRIVE_MOTOR_1_IN2   3
#define SL_CR_PIN_DRIVE_MOTOR_1_FAULT SL_CR_PIN_INVALID
#define SL_CR_PIN_DRIVE_MOTOR_1_SLEEP 5
#define SL_CR_PIN_DRIVE_MOTOR_2_IN1   6
#define SL_CR_PIN_DRIVE_MOTOR_2_IN2   7
#define SL_CR_PIN_DRIVE_MOTOR_2_FAULT SL_CR_PIN_INVALID
#define SL_CR_PIN_DRIVE_MOTOR_2_SLEEP 9
#define SL_CR_PIN_ONBOARD_LED         13
#define SL_CR_PIN_DRIVE_ENCODER_1_A   20
#define SL_CR_PIN_DRIVE_ENCODER_1_B   21
#define SL_CR_PIN_DRIVE_ENCODER_2_A   22
#define SL_CR_PIN_DRIVE_ENCODER_2_B   23
/////////////////////////////////////////////////////////////////
////////////////// PWM Global Config ////////////////////////////
/* PWM Resolution */
#define SL_CR_PWM_RESOLUTION 10
#define SL_CR_PWM_MAX_VALUE  ((1<<SL_CR_PWM_RESOLUTION)-1)
/* Default frequency to use for PWM pins (hz) */
#define SL_CR_DEFAULT_PWM_FREQ 23437.5
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
/////////////////////// FreeRTOS ////////////////////////////////
/* Default stack size to use for FreeRTOS Tasks (in words) */
#define SL_CR_LOG_TASK_STACK_SIZE 256
#define SL_CR_DEFAULT_TASK_STACK_SIZE 256
/* Loop Task stack sizes */
#define SL_CR_CONTROL_LOOP_TASK_STACK_SIZE SL_CR_DEFAULT_TASK_STACK_SIZE
#ifdef _VIRTUAL_MOTORS_
/* Larger stack required for strings */
#undef  SL_CR_CONTROL_LOOP_TASK_STACK_SIZE
#define SL_CR_CONTROL_LOOP_TASK_STACK_SIZE 2048
#endif
/////////////////////////////////////////////////////////////////

#endif /* __SL_CR_CONFIG_H__ */