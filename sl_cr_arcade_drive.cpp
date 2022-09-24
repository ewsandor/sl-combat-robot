/*
  sl_cr_arcade_drive.cpp
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#include "sl_cr_arcade_drive.hpp"
#include "sl_cr_sbus.hpp"

using namespace sandor_laboratories::robot;

void sl_cr_arcade_drive_c::init()
{
  this->left_motor       = nullptr;
  this->right_motor      = nullptr;
  this->throttle_channel = SL_CR_RC_CH_INVALID;
  this->steering_channel = SL_CR_RC_CH_INVALID;

  this->deadzone         = SL_CR_ARCADE_DRIVE_DEFAULT_DEADZONE;

  this->failsafe   = nullptr;
}

sl_cr_arcade_drive_c::sl_cr_arcade_drive_c
(
  sandor_laboratories::robot::motor_driver_c* left_motor, 
  sandor_laboratories::robot::motor_driver_c* right_motor, 
  sl_cr_rc_channel_t throttle_channel, 
  sl_cr_rc_channel_t steering_channel
)
{
  init();

  this->left_motor       = left_motor;
  this->right_motor      = right_motor;
  this->throttle_channel = throttle_channel;
  this->steering_channel = steering_channel;
}

bool sl_cr_arcade_drive_c::disabled()
{
  bool ret_val = false;

  if(failsafe != nullptr &&
     failsafe(failsafe_user_data_ptr) == true )
  {
    ret_val = true;
  }

  return ret_val;
}

rpm_t sl_cr_arcade_drive_c::get_motor_speed_from_rc_value(sl_cr_rc_channel_value_t rc_value, const motor_driver_c* motor)
{
  rpm_t new_speed = 0;

  if((rc_value > (SL_CR_RC_CH_CENTER_VALUE+deadzone)) ||
      (rc_value < (SL_CR_RC_CH_CENTER_VALUE-deadzone)))
  {
    /* Input is not within deadzone, scale motor to input */
    const rpm_t input_range = SL_CR_RC_CH_MAX_VALUE-SL_CR_RC_CH_MIN_VALUE;
    const rpm_t motor_range = motor->get_max_rpm()-motor->get_min_rpm();

    new_speed = (((rc_value-SL_CR_RC_CH_MIN_VALUE)*motor_range)/input_range)+motor->get_min_rpm();
  }

  return new_speed;
}

void sl_cr_arcade_drive_c::set_motor_speeds()
{
  const sl_cr_rc_channel_value_t throttle_raw = sl_cr_sbus_get_ch_value(throttle_channel);
  const sl_cr_rc_channel_value_t steering_raw = sl_cr_sbus_get_ch_value(steering_channel);

  if(!SL_CR_RC_CH_VALUE_VALID(throttle_raw) || !SL_CR_RC_CH_VALUE_VALID(steering_raw))
  {
    /* Invalid input, disable motor */
    left_motor->disable(MOTOR_DISABLE_DRIVE_STRATEGY);
    right_motor->disable(MOTOR_DISABLE_DRIVE_STRATEGY);
  }
  else
  {
    rpm_t left_motor_speed  = get_motor_speed_from_rc_value(throttle_raw, left_motor)  + get_motor_speed_from_rc_value(steering_raw, left_motor);
    rpm_t right_motor_speed = get_motor_speed_from_rc_value(throttle_raw, right_motor) - get_motor_speed_from_rc_value(steering_raw, right_motor);

    left_motor_speed = (left_motor_speed > left_motor->get_max_rpm())?left_motor->get_max_rpm():left_motor_speed;
    left_motor_speed = (left_motor_speed < left_motor->get_min_rpm())?left_motor->get_min_rpm():left_motor_speed;
    right_motor_speed = (right_motor_speed > right_motor->get_max_rpm())?right_motor->get_max_rpm():right_motor_speed;
    right_motor_speed = (right_motor_speed < right_motor->get_min_rpm())?right_motor->get_min_rpm():right_motor_speed;

    left_motor->change_set_rpm(left_motor_speed);
    right_motor->change_set_rpm(right_motor_speed);

    /* Input is valid, enable motor */
    left_motor->enable(MOTOR_DISABLE_DRIVE_STRATEGY);
    right_motor->enable(MOTOR_DISABLE_DRIVE_STRATEGY);
  }
}

void sl_cr_arcade_drive_c::loop()
{
  if(disabled())
  {
    /* Drive disabled, stop motors */
    left_motor->disable(MOTOR_DISABLE_DRIVE_STRATEGY);
    right_motor->disable(MOTOR_DISABLE_DRIVE_STRATEGY);
  }
  else
  {
    /* Set motor speeds */
    set_motor_speeds();
  }
}