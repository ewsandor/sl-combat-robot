/*
  sl_cr_control_loop.hpp
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#include "sl_cr_control_loop.hpp"

using namespace sandor_laboratories::combat_robot;

template <typename SETPOINT_T, typename OUTPUT_T>
inline void sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::set_initial_state()
{
  sp     = (sp_min+sp_max)/2;
  output = (output_min+output_max)/2;
  error  = 0;
}

template <typename SETPOINT_T, typename OUTPUT_T>
sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::sl_cr_control_loop_c(SETPOINT_T sp_min,     SETPOINT_T sp_max,
                                                                 OUTPUT_T output_min,   OUTPUT_T output_max, 
                                                                 log_key_e log_key)
  : sp_min(sp_min), sp_max(sp_max), output_min(output_min), output_max(output_max), log_key(log_key)
{
  set_initial_state();
}
template <typename SETPOINT_T, typename OUTPUT_T>
sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::sl_cr_control_loop_c(SETPOINT_T sp_min, SETPOINT_T sp_max, 
                                                                 log_key_e log_key)
  : sl_cr_control_loop_c(sp_min, sp_max, sp_min, sp_max, log_key) {}

template <typename SETPOINT_T, typename OUTPUT_T>
bool sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::set_setpoint(SETPOINT_T new_setpoint) 
{
  bool ret_val = true;
  if(new_setpoint > get_sp_max())
  {
    sp = get_sp_max();
    ret_val = false;
  }
  else if(new_setpoint < get_sp_min())
  {
    sp = get_sp_min();
    ret_val = false;
  }
  else
  {
    sp = new_setpoint;
  }
  return ret_val;
}
template <typename SETPOINT_T, typename OUTPUT_T>
bool sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::set_output(OUTPUT_T new_output)
{
  bool ret_val = true;
  if(new_output > get_output_max())
  {
    output = get_output_max();
    ret_val = false;
  }
  else if(new_output < get_output_min())
  {
    output = get_output_min();
    ret_val = false;
  }
  else
  {
    output = new_output;
  }
  return ret_val;
}

template <typename SETPOINT_T, typename OUTPUT_T>
OUTPUT_T sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::loop(SETPOINT_T feedback) 
{
  error = (this->get_setpoint() - feedback);
  update_output();

  SL_CR_LOG_SNPRINTF(get_log_key(), LOG_LEVEL_DEBUG_4, "|%+05d|%+05d|%+05d|", this->get_setpoint(), get_output(), get_error());

  return get_output();
}


template <typename SETPOINT_T, typename OUTPUT_T>
void sl_cr_control_loop_c<SETPOINT_T, OUTPUT_T>::reset(SETPOINT_T new_setpoint)
{
  set_initial_state();
  set_setpoint(new_setpoint);
}