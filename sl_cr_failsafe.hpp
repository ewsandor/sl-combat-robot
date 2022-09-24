/*
  sl_cr_failsafe.hpp
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#ifndef __SL_CR_FAILSAFE_HPP__
#define __SL_CR_FAILSAFE_HPP__

namespace sandor_laboratories
{
  namespace robot
  {
    namespace combat
    {
      typedef enum
      {
        /* Initial failsafe set at bootup */
        FAILSAFE_BOOT,
        /* Do not arm unless arming switch first entered disarmed state.  (Avoid unexpected arming if arm switch is active at startup) */
        FAILSAFE_ARM_SWITCH_DISARM,
        /* Disabled by arming switch */
        FAILSAFE_ARM_SWITCH,
        /* SBUS in failsafe mode */
        FAILSAFE_SBUS,
        /* SBUS data is stale */
        FAILSAFE_SBUS_STALE,
        /* Max failsafe value */
        FAILSAFE_SBUS_MAX,
        /* Invalid failsafe value */
        FAILSAFE_SBUS_INVLAID
      } failsafe_reason_e;

      #define SL_CR_FAILSAFE_INVALID_MASK (FAILSAFE_SBUS_INVLAID-1)

      typedef unsigned int failsafe_mask_t;

      /* Sets failsafe reason as active */
      void set_failsafe_mask(failsafe_reason_e reason);
      /* Clears failsafe reason */
      void clear_failsafe_mask(failsafe_reason_e reason);
      /* Set failsafe reason based on boolean value */
      void set_failsafe_mask_value(failsafe_reason_e reason, bool value);
      /* Returns current failsafe mask */
      failsafe_mask_t get_failsafe_mask();
      /* Return boolean if failsafe is active */
      bool get_failsafe_set();
      /* Return boolean if failsafe is active for a specific reason*/
      bool get_failsafe_set(failsafe_reason_e reason);

      extern const void* failsafe_check_user_data_ptr;
      bool failsafe_check(const void*);

      /* Time (in ms) failsafe may be active until re-arming the arm switch is required 
        (allow short, self-correcting failsafes to correct, long events require explict user re-arm) */
      #define SL_CR_FAILSAFE_ARMSWITCH_REARM_TIMEOUT 200
      /* Maximum number of self-correcting failsafe events until rearm is required */
      #define SL_CR_FAILSAFE_REPEAT_REARM_THRESHOLD 5
      /* Arm switch maintenance, to be called every loop */
      void failsafe_armswitch_loop();
    }
  }
}

#endif /* __SL_CR_FAILSAFE_HPP__ */