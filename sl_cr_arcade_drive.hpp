/*
  sl_cr_arcade_drive.hpp
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#ifndef __SL_CR_ARCADE_DRIVE_HPP__
#define __SL_CR_ARCADE_DRIVE_HPP__

#include "sl_robot_motor_driver.hpp"
#include "sl_cr_types.hpp"

/* RC channel values < CENTER+DEADZONE && > CENTER-DEADZONE will be treated as neutral */
#define SL_CR_ARCADE_DRIVE_DEFAULT_DEADZONE 50

class sl_cr_arcade_drive_c
{
  private:
    sandor_laboratories::robot::motor_driver_c *left_motor;
    sandor_laboratories::robot::motor_driver_c *right_motor;
    sl_cr_rc_channel_t  throttle_channel;
    sl_cr_rc_channel_t  steering_channel;

    /* RC channel values < CENTER+DEADZONE && > CENTER-DEADZONE will be treated as neutral */
    sl_cr_rc_channel_value_t  deadzone;

    const void * failsafe_user_data_ptr = nullptr;
    /* Function pointer to call to check if failsafe has been triggered */
    sandor_laboratories::robot::failsafe_f failsafe;

    /* Initializes class with default values */
    void init();

    /* Gets rpm value within motor range proportional to RC value */
    sandor_laboratories::robot::rpm_t get_motor_speed_from_rc_value(sl_cr_rc_channel_value_t rc_value, const sandor_laboratories::robot::motor_driver_c* motor);

    /* Compute motor speeds */
    void set_motor_speeds();

  public:
    sl_cr_arcade_drive_c
    (
      sandor_laboratories::robot::motor_driver_c* left_motor, 
      sandor_laboratories::robot::motor_driver_c* right_motor, 
      sl_cr_rc_channel_t throttle_channel, 
      sl_cr_rc_channel_t steering_channel
    );

    /* Checks if drive is currently disabled */
    bool disabled();
    
    /* To be called regularly as part of the software's main loop */
    void loop();
};

#endif /*__SL_CR_ARCADE_DRIVE_HPP__*/