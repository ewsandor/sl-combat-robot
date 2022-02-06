#include "sl_cr_motor_driver.hpp"


void sl_cr_motor_driver_c::init()
{
  failsafe_check = nullptr;
  invert_direction = false;
  speed = 0;
  min_speed = SL_CR_MOTOR_DRIVER_DEFAULT_MIN_SPEED;
  max_speed = SL_CR_MOTOR_DRIVER_DEFAULT_MAX_SPEED;
}

sl_cr_motor_driver_c::sl_cr_motor_driver_c()
{
  init();
}

sl_cr_motor_driver_c::sl_cr_motor_driver_c(sl_cr_failsafe_f * failsafe_check)
{
  init();
  this->failsafe_check = failsafe_check;
}

sl_cr_motor_driver_speed_t sl_cr_motor_driver_c::get_min_speed()
{
  return min_speed;
}
sl_cr_motor_driver_speed_t sl_cr_motor_driver_c::get_max_speed()
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

  speed = new_speed;
}

bool sl_cr_motor_driver_c::disabled()
{
  bool ret_val = false;
  if(failsafe_check != nullptr &&
     true == (*failsafe_check)())
  {
    ret_val = true;
  }

  return ret_val;
}

void sl_cr_motor_driver_c::loop()
{
  if(disabled())
  {
    disable_motor();
  }
  else
  {
    command_motor();
  }
}