/*
  sl_cr_drive.hpp 
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  April 2022
*/

#ifndef __SL_CR_DRIVE_HPP__
#define __SL_CR_DRIVE_HPP__

#ifdef _ARCADE_DRIVE_
#include "sl_cr_arcade_drive.hpp"
#else
#include "sl_cr_tank_drive.hpp"
#endif
#include "sl_robot_encoder.hpp"
#include "sl_robot_motor_driver.hpp"
#include "sl_robot_pid_loop.hpp"

/* Loop Task periods in ms */
#define SL_CR_CONTROL_LOOP_PERIOD 10
#define SL_CR_DRIVE_PERIOD 10

#define SL_CR_MOTOR_DRIVER_REAL_MAX_RPM 1000
#define SL_CR_MOTOR_DRIVER_REAL_MIN_RPM (-SL_CR_MOTOR_DRIVER_REAL_MAX_RPM)

typedef struct
{
  sandor_laboratories::robot::motor_driver_c                              *driver;
  sandor_laboratories::robot::encoder_c                                   *encoder;
  sandor_laboratories::robot::control_loop_c
    <sandor_laboratories::robot::rpm_t,sandor_laboratories::robot::rpm_t> *control_loop;
} sl_cr_drive_motor_stack_s;

typedef struct 
{
  /* Physical Motor Stacks */
  sl_cr_drive_motor_stack_s left_motor_stack;
  sl_cr_drive_motor_stack_s right_motor_stack;

  /* Drive Strategy */
  #ifdef _ARCADE_DRIVE_
  sl_cr_arcade_drive_c *arcade_drive;
  #else
  sl_cr_tank_drive_c *tank_drive;
  #endif

} sl_cr_drive_data_s;

/* Initializes data for drive data */
const sl_cr_drive_data_s *sl_cr_drive_init();

/* Registers required drive interrupts */
void sl_cr_drive_register_interrupts();

/* Loop to manage physical control loops */
void sl_cr_drive_control_loop();

/* Loop to manage higher level drive strategy */
void sl_cr_drive_strategy_loop();

#endif /* __SL_CR_DRIVE_HPP__ */