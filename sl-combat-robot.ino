/*
  sl-combat-robot.ino
  Sandor Laboratories Combat Robot Software
  Edward Sandor
  February 2022
*/

#include <Arduino.h>
#include <Watchdog_t4.h>
#include <arduino_freertos.h>

#include "sl_cr_config.h"
#include "sl_cr_drive.hpp"
#include "sl_cr_failsafe.hpp"
#include "sl_cr_sbus.hpp"
#include "sl_cr_types.hpp"
#include "sl_cr_version.h"

#include "sl_robot_imu.hpp"
#include "sl_robot_log_task.hpp"
#include "sl_robot_utils.hpp"

using namespace sandor_laboratories::robot;

const sl_cr_drive_data_s *drive_data_ptr = nullptr;

time_ms_t watchdog_fed;
/* Watchdog Timeout in ms, 32ms to 522.232s */
#define SL_CR_WATCHDOG_TIMEOUT 100
/* Watchdog feeding schedule in ms */
#define SL_CR_WATCHDOG_FEEDING_SCHEDULE (SL_CR_WATCHDOG_TIMEOUT / 2)
/* Watchdog window in ms, 32ms to 522.232s, must be smaller than timeout */
#define SL_CR_WATCHDOG_WINDOW (SL_CR_WATCHDOG_FEEDING_SCHEDULE - 1)
WDT_T4<WDT3> wdt;
void wdt_callback()
{
  Serial.println("WATCHDOG ABOUT TO EXPIRE...");
  Serial.print(String(millis() - watchdog_fed));
  Serial.println("ms since last fed...");
  Serial.flush();
}

/* SBUS read frequency in ms */
#define SL_CR_SBUS_PERIOD SL_CR_SBUS_UPDATE_PERIOD

static void control_loop_task(void *)
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(SL_CR_CONTROL_LOOP_PERIOD);
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    sl_cr_drive_control_loop();
  }
}

static void drive_task(void *)
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(SL_CR_DRIVE_PERIOD);
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    sl_cr_drive_strategy_loop();
  }
}

static void watchdog_task(void *)
{
  const TickType_t xPeriod = pdMS_TO_TICKS(SL_CR_WATCHDOG_FEEDING_SCHEDULE);

  for (;;)
  {
    vTaskDelay(xPeriod);

    /* Feed watchdog */
    wdt.feed();
    watchdog_fed = millis();
  }
}

static void sbus_task(void *)
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(SL_CR_SBUS_PERIOD);
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    /* Read any new SBUS data */
    sl_cr_sbus_loop();
    /* Check if ARM switch is set */
    combat::failsafe_armswitch_loop();
  }
}

TaskHandle_t log_task_handle = nullptr;
static void log_task(void *)
{
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "######### BOOTUP END #########");
  for(;;)
  {
    log_flush();
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
  }
}

#ifdef _SERIAL_DEBUG_MODE_
static void serial_debug_task(void *)
{
  TickType_t xLastWakeTime;
  const TickType_t xPeriod = pdMS_TO_TICKS(1000);
  xLastWakeTime = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);

    if(drive_data_ptr->left_motor_stack.control_loop)
    {
      SL_CR_LOG_SNPRINTF(LOG_KEY_DEBUG_TASK, LOG_LEVEL_DEBUG, 
        "Left motor set_rpm: %u motor commanded_rpm: %u encoder rpm: %u error: %u.", 
        drive_data_ptr->left_motor_stack.driver->get_set_rpm(),
        drive_data_ptr->left_motor_stack.driver->get_commanded_rpm(),
        drive_data_ptr->left_motor_stack.driver->get_real_rpm(),
        drive_data_ptr->left_motor_stack.control_loop->get_error());
    }
    else
    {
      SL_CR_LOG_SNPRINTF(LOG_KEY_DEBUG_TASK, LOG_LEVEL_DEBUG, 
        "Left motor set_rpm: %u motor commanded_rpm: %u encoder rpm: %u", 
        drive_data_ptr->left_motor_stack.driver->get_set_rpm(),
        drive_data_ptr->left_motor_stack.driver->get_commanded_rpm(),
        drive_data_ptr->left_motor_stack.driver->get_real_rpm());
    }

    if(drive_data_ptr->right_motor_stack.control_loop)
    {
      SL_CR_LOG_SNPRINTF(LOG_KEY_DEBUG_TASK, LOG_LEVEL_DEBUG, 
        "Reft motor set_rpm: %u motor commanded_rpm: %u encoder rpm: %u error: %u.", 
        drive_data_ptr->right_motor_stack.driver->get_set_rpm(),
        drive_data_ptr->right_motor_stack.driver->get_commanded_rpm(),
        drive_data_ptr->right_motor_stack.driver->get_real_rpm(),
        drive_data_ptr->right_motor_stack.control_loop->get_error());
    }
    else
    {
      SL_CR_LOG_SNPRINTF(LOG_KEY_DEBUG_TASK, LOG_LEVEL_DEBUG, 
        "Reft motor set_rpm: %u motor commanded_rpm: %u encoder rpm: %u", 
        drive_data_ptr->right_motor_stack.driver->get_set_rpm(),
        drive_data_ptr->right_motor_stack.driver->get_commanded_rpm(),
        drive_data_ptr->right_motor_stack.driver->get_real_rpm());
    }
  }
}
#endif


/* IMU LOGIC */
imu_c *imu;

void setup()
{
  /* Enable Bootup LED */
  pinMode(SL_CR_PIN_ONBOARD_LED, arduino::OUTPUT);
  digitalWrite(SL_CR_PIN_ONBOARD_LED, arduino::HIGH);

  /* Serial for debug logging */
#ifdef _SERIAL_DEBUG_MODE_
  Serial.begin(115200);
  while (!Serial)
  {
  }
#endif
  log_init(&log_task_handle, SL_CR_ACTIVE_LOG_LEVEL);
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "Log Initialized.");

  /* Start of Bootup */
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO,"######## BOOTUP START ########");

  /* Log software details */
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, SL_CR_SOFTWARE_INTRO);

  /* Configure Watchdog Timer before anything else */
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "Activating Watchdog.");
  WDT_timings_t wdt_config;
  wdt_config.window = SL_CR_WATCHDOG_WINDOW;
  wdt_config.timeout = SL_CR_WATCHDOG_TIMEOUT;
  wdt_config.callback = wdt_callback;
  watchdog_fed = millis();
  wdt.begin(wdt_config);

  log_snprintf(LOG_KEY_BOOT, LOG_LEVEL_INFO, "Failsafe mask: 0x%x", combat::get_failsafe_mask());

  /* Configure PWM resolution */
  analogWriteResolution(SL_CR_PWM_RESOLUTION);

  /* Begin the SBUS communication */
  sl_cr_sbus_init();
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "SBUS Configured.");

  drive_data_ptr = sl_cr_drive_init();
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "Drive Configured.");

  /* Init IMU */
  imu = new imu_c();
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "IMU Configured.");

  /* Configure FreeRTOS */
  #ifdef _SERIAL_DEBUG_MODE_
  xTaskCreate(serial_debug_task, "Serial Debug Task", SL_CR_DEFAULT_TASK_STACK_SIZE,      nullptr, 1, nullptr);
  #endif
  xTaskCreate(log_task,          "Log Task",          SL_CR_LOG_TASK_STACK_SIZE,          nullptr, 0, &log_task_handle);
  xTaskCreate(watchdog_task,     "Watchdog Task",     SL_CR_DEFAULT_TASK_STACK_SIZE,      nullptr, 2, nullptr);
  xTaskCreate(drive_task,        "Drive Task",        SL_CR_DEFAULT_TASK_STACK_SIZE,      nullptr, 5, nullptr);
  xTaskCreate(sbus_task,         "SBUS Task",         SL_CR_DEFAULT_TASK_STACK_SIZE,      nullptr, 6, nullptr);
  xTaskCreate(control_loop_task, "Control Loop Task", SL_CR_CONTROL_LOOP_TASK_STACK_SIZE, nullptr, 7, nullptr);
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "FreeRTOS Configured.");

  /* Bootup Complete */
  /* Clear Bootup LED */
  digitalWrite(SL_CR_PIN_ONBOARD_LED, arduino::LOW);
  /* Clear Bootup failsafe */
  combat::clear_failsafe_mask(combat::FAILSAFE_BOOT);

  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "Attaching Pin Interrupts.");
  sl_cr_drive_register_interrupts();
  log_cstring(LOG_KEY_BOOT, LOG_LEVEL_INFO, "Starting FreeRTOS scheduler.");

  vTaskStartScheduler();
}

void loop() {}