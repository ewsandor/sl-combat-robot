/*
  sl_cr_motor_driver.cpp
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#include "sl_cr_motor_driver.hpp"

#define SL_CR_DISABLE_BIT(reason) (1 << reason)

void sl_cr_motor_driver_c::init()
{
  failsafe_check   = nullptr;
  disable_mask     = 0x0;
  invert_direction = false;
  speed            = 0;
  target_speed     = 0;
  min_speed = SL_CR_MOTOR_DRIVER_DEFAULT_MIN_SPEED;
  max_speed = SL_CR_MOTOR_DRIVER_DEFAULT_MAX_SPEED;
}

sl_cr_motor_driver_c::sl_cr_motor_driver_c()
{
  init();
}

sl_cr_motor_driver_c::sl_cr_motor_driver_c(sl_cr_failsafe_f failsafe_check)
{
  init();
  this->failsafe_check = failsafe_check;
}

sl_cr_motor_driver_speed_t sl_cr_motor_driver_c::get_min_speed() const
{
  return min_speed;
}
sl_cr_motor_driver_speed_t sl_cr_motor_driver_c::get_neutral_speed() const
{
  return ((min_speed+max_speed)/2);
}
sl_cr_motor_driver_speed_t sl_cr_motor_driver_c::get_max_speed() const
{
  return max_speed;
}

void sl_cr_motor_driver_c::set_speed(sl_cr_motor_driver_speed_t new_speed)
{
  if(new_speed > SL_CR_MOTOR_DRIVER_DEFAULT_MAX_SPEED)
  {
    new_speed = SL_CR_MOTOR_DRIVER_DEFAULT_MAX_SPEED;
  }
  if(new_speed < SL_CR_MOTOR_DRIVER_DEFAULT_MIN_SPEED)
  {
    new_speed = SL_CR_MOTOR_DRIVER_DEFAULT_MIN_SPEED;
  }

  if(invert_direction)
  {
    new_speed = max_speed+min_speed-new_speed;
  }

  target_speed = new_speed;
}

void sl_cr_motor_driver_c::brake_motor()
{
  set_speed(get_neutral_speed());
}

void sl_cr_motor_driver_c::disable(sl_cr_motor_disable_reason_e reason)
{
  disable_mask |= SL_CR_DISABLE_BIT(reason);
  disable_motor();
}
void sl_cr_motor_driver_c::enable(sl_cr_motor_disable_reason_e reason)
{
  disable_mask &= ~(SL_CR_DISABLE_BIT(reason));
}
bool sl_cr_motor_driver_c::disabled() const
{
  bool ret_val = false;

  if((disable_mask != 0) ||
     (failsafe_check != nullptr && failsafe_check() == true))
  {
    ret_val = true;
  }

  return ret_val;
}

sl_cr_motor_driver_fault_status_e sl_cr_motor_driver_c::get_fault_status() const
{
  return SL_CR_MOTOR_DRIVER_FAULT_STATUS_UNKNOWN;
}

void sl_cr_motor_driver_c::loop()
{
  if(disabled())
  {
    speed = get_neutral_speed();
    disable_motor();
  }
  else
  {
    speed = target_speed;
    command_motor();
  }
}