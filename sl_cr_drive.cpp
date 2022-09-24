/*
  sl_cr_drive.cpp 
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  April 2022
*/

#include <Arduino.h>

#include "sl_cr_config.h"
#include "sl_cr_drive.hpp"

#ifdef _VIRTUAL_MOTORS_
#include "sl_robot_motor_driver_virtual.hpp"
#else
#include "sl_robot_motor_driver_drv8256p.hpp"
#endif
#include "sl_cr_failsafe.hpp"
#include "sl_robot_utils.hpp"

using namespace sandor_laboratories::robot;

const pid_loop_params_s pid_params = 
{
  .p_num = 50,
  .p_den = 100,
  .i_num = 25,
  .i_den = 100,
  .d_num = 12,
  .d_den = 100,
};

const pwm_config_s pwm_config
{
  .frequency  = SL_CR_DEFAULT_PWM_FREQ,
  .resolution = SL_CR_PWM_RESOLUTION,
  .max_value  = SL_CR_PWM_MAX_VALUE
};

const pin_t right_motor_in1_pin   = SL_CR_PIN_DRIVE_MOTOR_1_IN1;
const pin_t right_motor_in2_pin   = SL_CR_PIN_DRIVE_MOTOR_1_IN2;
const pin_t right_motor_sleep_pin = SL_CR_PIN_DRIVE_MOTOR_1_SLEEP;
const pin_t right_encoder_a_pin   = SL_CR_PIN_DRIVE_ENCODER_1_A;
const pin_t right_encoder_b_pin   = SL_CR_PIN_DRIVE_ENCODER_1_B;
const pin_t left_motor_in1_pin    = SL_CR_PIN_DRIVE_MOTOR_2_IN1;
const pin_t left_motor_in2_pin    = SL_CR_PIN_DRIVE_MOTOR_2_IN2;
const pin_t left_motor_sleep_pin  = SL_CR_PIN_DRIVE_MOTOR_2_SLEEP;
const pin_t left_encoder_a_pin    = SL_CR_PIN_DRIVE_ENCODER_2_A;
const pin_t left_encoder_b_pin    = SL_CR_PIN_DRIVE_ENCODER_2_B;

/* Drive Data */
sl_cr_drive_data_s drive_data = {0};

/* Encoder interrupts */
void interrupt_left_encoder_a()
{
  if(drive_data.left_motor_stack.encoder)
  {
    critical_section_enter_interrupt();
    drive_data.left_motor_stack.encoder->sample_channel_a();
    critical_section_exit_interrupt();
  }
}
void interrupt_left_encoder_b()
{
  if(drive_data.left_motor_stack.encoder)
  {
    critical_section_enter_interrupt();
    drive_data.left_motor_stack.encoder->sample_channel_b();
    critical_section_exit_interrupt();
  }
}
void interrupt_right_encoder_a()
{
  if(drive_data.right_motor_stack.encoder)
  {
    critical_section_enter_interrupt();
    drive_data.right_motor_stack.encoder->sample_channel_a();
    critical_section_exit_interrupt();
  }
}
void interrupt_right_encoder_b()
{
  if(drive_data.right_motor_stack.encoder)
  {
    critical_section_enter_interrupt();
    drive_data.right_motor_stack.encoder->sample_channel_b();
    critical_section_exit_interrupt();
  }
}

void sl_cr_drive_init_motor_stacks()
{
  motor_driver_config_s drive_motor_config;
  motor_driver_c::init_config(&drive_motor_config);
  drive_motor_config.failsafe = combat::failsafe_check;
  drive_motor_config.min_rpm  = SL_CR_MOTOR_DRIVER_REAL_MIN_RPM;
  drive_motor_config.max_rpm  = SL_CR_MOTOR_DRIVER_REAL_MAX_RPM;

#ifdef _VIRTUAL_MOTORS_
  drive_motor_config.min_commanded_rpm = -100;
  drive_motor_config.max_commanded_rpm =  100;
  drive_motor_config.log_key      = LOG_KEY_MOTOR_DRIVER_LEFT;
  drive_data.left_motor_stack.driver = new sl_cr_motor_driver_virtual_c("Left Motor", drive_motor_config);
  drive_motor_config.log_key      = LOG_KEY_MOTOR_DRIVER_RIGHT;
  drive_data.right_motor_stack.driver = new sl_cr_motor_driver_virtual_c("Right Motor", drive_motor_config);
#else
  drive_motor_config.min_commanded_rpm = -SL_CR_PWM_MAX_VALUE;
  drive_motor_config.max_commanded_rpm =  SL_CR_PWM_MAX_VALUE;

  drive_data.left_motor_stack.control_loop = new pid_loop_c<rpm_t,rpm_t>
  (
    drive_motor_config.min_rpm, drive_motor_config.max_rpm, 
    drive_motor_config.min_commanded_rpm, drive_motor_config.max_commanded_rpm,
    pid_params, LOG_KEY_MOTOR_CONTROL_LOOP_LEFT
  );
  drive_data.right_motor_stack.control_loop = new pid_loop_c<rpm_t,rpm_t>
  (
    drive_motor_config.min_rpm, drive_motor_config.max_rpm, 
    drive_motor_config.min_commanded_rpm, drive_motor_config.max_commanded_rpm,
    pid_params, LOG_KEY_MOTOR_CONTROL_LOOP_RIGHT
  );

  /* Left Motor */
  drive_data.left_motor_stack.encoder = new encoder_c(left_encoder_a_pin, left_encoder_b_pin,false,12,1,30);
  drive_motor_config.log_key      = LOG_KEY_MOTOR_DRIVER_LEFT;
  drive_motor_config.encoder      = drive_data.left_motor_stack.encoder;
  drive_motor_config.control_loop = drive_data.left_motor_stack.control_loop;
  drive_data.left_motor_stack.driver  = new motor_driver_drv8256p_c(left_motor_sleep_pin, left_motor_in1_pin, left_motor_in2_pin, pwm_config, drive_motor_config);

  /* Right Motor */
  drive_data.right_motor_stack.encoder = new encoder_c(right_encoder_a_pin, right_encoder_b_pin,true,12,1,30);
  drive_motor_config.log_key      = LOG_KEY_MOTOR_DRIVER_RIGHT;
  drive_motor_config.encoder      = drive_data.right_motor_stack.encoder;
  drive_motor_config.control_loop = drive_data.right_motor_stack.control_loop;
  drive_data.right_motor_stack.driver = new motor_driver_drv8256p_c(right_motor_sleep_pin, right_motor_in1_pin, right_motor_in2_pin, pwm_config, drive_motor_config);
#endif

  #ifdef _FORCE_LIMP_MODE_
  drive_data.left_motor_stack.driver->set_limp_mode(true);
  drive_data.right_motor_stack.driver->set_limp_mode(true);
  #endif
}

const sl_cr_drive_data_s *sl_cr_drive_init()
{
  /* Init Motor Stacks */
  sl_cr_drive_init_motor_stacks();

  /* Init Drive Strategy */
#ifdef _ARCADE_DRIVE_
  drive_data.arcade_drive = new sl_cr_arcade_drive_c(drive_data.left_motor_stack.driver, drive_data.right_motor_stack.driver, SL_CR_ARCADE_DRIVE_THROTTLE_CH, SL_CR_ARCADE_DRIVE_STEERING_CH);
#else
  drive_data.tank_drive = new sl_cr_tank_drive_c(drive_data.left_motor_stack.driver, drive_data.right_motor_stack.driver, SL_CR_TANK_DRIVE_LEFT_CH, SL_CR_TANK_DRIVE_RIGHT_CH);
#endif

  return &drive_data;
}

void sl_cr_drive_register_interrupts()
{
  if(drive_data.left_motor_stack.encoder)
  {
    attachInterrupt(left_encoder_a_pin, interrupt_left_encoder_a, arduino::CHANGE);
    attachInterrupt(left_encoder_b_pin, interrupt_left_encoder_b, arduino::CHANGE);
  }

  if(drive_data.right_motor_stack.encoder)
  {
    attachInterrupt(right_encoder_a_pin, interrupt_right_encoder_a, arduino::CHANGE);
    attachInterrupt(right_encoder_b_pin, interrupt_right_encoder_b, arduino::CHANGE);
  }
}

void sl_cr_drive_motor_stack_control_loop(sl_cr_drive_motor_stack_s *motor_stack)
{
  if(motor_stack->encoder)
  {
    motor_stack->encoder->loop();
  }
  motor_stack->driver->loop();
}

void sl_cr_drive_control_loop()
{
  sl_cr_drive_motor_stack_control_loop(&drive_data.left_motor_stack);
  sl_cr_drive_motor_stack_control_loop(&drive_data.right_motor_stack);
}

void sl_cr_drive_strategy_loop()
{
  #ifdef _ARCADE_DRIVE_
    /* Arcade Drive loop */
    drive_data.arcade_drive->loop();
  #else
    /* Tank Drive loop */
    drive_data.tank_drive->loop();
  #endif
}