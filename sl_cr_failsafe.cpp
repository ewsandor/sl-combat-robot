/*
  sl_cr_failsafe.cpp
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#include <Arduino.h>

#include "sl_cr_failsafe.hpp"
#include "sl_robot_log.hpp"
#include "sl_cr_sbus.hpp"
#include "sl_robot_utils.hpp"

using namespace sandor_laboratories::robot;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Failsafe Utilities */
#define SL_CR_FAILSAFE_BIT(reason) (1 << reason)

#define SL_CR_ARM_SWITCH_THRESHOLD ((SL_CR_RC_CH_MAX_VALUE*90)/100)

/* Default user data pointer for failsafe check */
void* failsafe_check_user_data_ptr = nullptr;

/* Master failsafe mask. Each bit corresponds to a reason from failsafe_mask_t.  
   Motors are to be powered if and only if mask is 0x0.
   Initialize with BOOT bit and ARM_SWITCH_DISARM bit such that nothing can activate until bootup is complete and the arm switch is cleared to avoid unexpected arming */
volatile combat::failsafe_mask_t sl_cr_failsafe_mask = SL_CR_FAILSAFE_BIT(combat::FAILSAFE_BOOT) | SL_CR_FAILSAFE_BIT(combat::FAILSAFE_ARM_SWITCH_DISARM);

/* Timeout timer reference time (last failsafe entry) to force rearming */
time_t armswitch_rearm_timeout_start = 0; //millis() starts from 0 at bootup
bool         rearm_timeout_expired = false;
/* Tracks number of failsafes since forcing rearm */
unsigned int repeat_failsafe_rearm_counter = 0;
bool         repeat_failsafe_rearm_required = false;

void combat::set_failsafe_mask(failsafe_reason_e reason)
{
  critical_section_enter();
  failsafe_mask_t old_failsafe_mask = sl_cr_failsafe_mask;
  sl_cr_failsafe_mask |= SL_CR_FAILSAFE_BIT(reason);
  failsafe_mask_t new_failsafe_mask = sl_cr_failsafe_mask;
  critical_section_exit();
  
  if(0 == (old_failsafe_mask & SL_CR_FAILSAFE_BIT(reason)))
  {
    log_snprintf(LOG_KEY_FAILSAFE, LOG_LEVEL_INFO, "New failsafe mask: 0x%x", new_failsafe_mask);

    if(0 == old_failsafe_mask)
    {
      log_cstring(LOG_KEY_FAILSAFE, LOG_LEVEL_INFO, "FAILSAFE SET!");
      /* Entered failsafe state, start rearm timer */
      armswitch_rearm_timeout_start = millis();
      rearm_timeout_expired = false;
      /* Count failsafe event */
      repeat_failsafe_rearm_counter++;
    }
  }
}
void combat::clear_failsafe_mask(failsafe_reason_e reason)
{
  critical_section_enter();
  failsafe_mask_t old_failsafe_mask = sl_cr_failsafe_mask;
  sl_cr_failsafe_mask &= ~(SL_CR_FAILSAFE_BIT(reason));
  failsafe_mask_t new_failsafe_mask = sl_cr_failsafe_mask;
  critical_section_exit();
  
  if(0 != (old_failsafe_mask & SL_CR_FAILSAFE_BIT(reason)))
  {
    log_snprintf(LOG_KEY_FAILSAFE, LOG_LEVEL_INFO, "New failsafe mask: 0x%x", new_failsafe_mask);
    if(0 == new_failsafe_mask)
    {
      log_cstring(LOG_KEY_FAILSAFE, LOG_LEVEL_INFO, "ARMED!");
    }
  }
}
void combat::set_failsafe_mask_value(failsafe_reason_e reason, bool value)
{
  if(value)
  {
    set_failsafe_mask(reason);
  }
  else
  {
    clear_failsafe_mask(reason);
  }
}
combat::failsafe_mask_t combat::get_failsafe_mask()
{
  failsafe_mask_t ret_val = SL_CR_FAILSAFE_INVALID_MASK;

  critical_section_enter();
  ret_val = sl_cr_failsafe_mask;
  critical_section_exit();

  return ret_val;
}
bool combat::get_failsafe_set()
{
  return (0 != get_failsafe_mask());
}
bool combat::get_failsafe_set(failsafe_reason_e reason)
{
  return (0 != (get_failsafe_mask() & SL_CR_FAILSAFE_BIT(reason)));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Failsafe Switch Management */

/* Only arm if prearm was triggered without arm switch armed (i.e. sequential pre-arm then arm) */
bool prearmswitch_first = false;
/* Only arm if both arm switches are first released */
bool armswitches_released_first = false;
void combat::failsafe_armswitch_loop()
{
  /* Check if arm switch is requsting robot to be armed */
  const sl_cr_rc_channel_value_t armswitch_raw = sl_cr_sbus_get_ch_value(SL_CR_ARM_SWITCH_CH);
  const bool armswitch_armed = SL_CR_RC_CH_VALUE_VALID(armswitch_raw) && (armswitch_raw > SL_CR_ARM_SWITCH_THRESHOLD);

  /* Check prearm switch state */
  const sl_cr_rc_channel_value_t prearmswitch_raw = sl_cr_sbus_get_ch_value(SL_CR_PREARM_SWITCH_CH);
  const bool prearmswitch_armed = SL_CR_RC_CH_VALUE_VALID(prearmswitch_raw) && (prearmswitch_raw > SL_CR_ARM_SWITCH_THRESHOLD);

  if(armswitches_released_first && prearmswitch_first && prearmswitch_armed && armswitch_armed)
  {
    /* Clear arm switch failsafe if pre-arm was set first and both pre-arm and arm switch are requesting arming */
    clear_failsafe_mask(FAILSAFE_ARM_SWITCH);
    /* Do not rearm until both switches are released */
    armswitches_released_first = false;
  }
  else if(!armswitch_armed)
  {
    /* If arm switch is not requesting arming, set failsafe */
    set_failsafe_mask(FAILSAFE_ARM_SWITCH);
  }

  /* Check if failsafe timeout is exceeded and force rearm */
  if(!rearm_timeout_expired &&
     get_failsafe_set() && 
     ((millis()-armswitch_rearm_timeout_start) > SL_CR_FAILSAFE_ARMSWITCH_REARM_TIMEOUT))
  {
    set_failsafe_mask(FAILSAFE_ARM_SWITCH_DISARM);
    rearm_timeout_expired = true;
  }
  if(!repeat_failsafe_rearm_required &&
      get_failsafe_set() &&
      repeat_failsafe_rearm_counter > SL_CR_FAILSAFE_REPEAT_REARM_THRESHOLD)
  {
    set_failsafe_mask(FAILSAFE_ARM_SWITCH_DISARM);
    repeat_failsafe_rearm_required = true;
  }
  /* If armswitch not requesting arming, clear disarm wait */
  if(SL_CR_RC_CH_VALUE_VALID(armswitch_raw) && !armswitch_armed)
  {
    clear_failsafe_mask(FAILSAFE_ARM_SWITCH_DISARM);
    /* Reset repeat failsafe counter */
    repeat_failsafe_rearm_counter = 0;
    repeat_failsafe_rearm_required = false;
  }

  /* Prearm is armed before arming switch is armed */
  prearmswitch_first = prearmswitch_armed && !armswitch_armed;
  if(!prearmswitch_armed && !armswitch_armed)
  {
    /* Both arm switches have been released */
    armswitches_released_first = true;
  }
}

bool combat::failsafe_check(const void*) 
{
  return get_failsafe_set();
};