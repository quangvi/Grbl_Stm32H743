/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h

#ifdef USER_STM32
  #include "Extern_Variables_functions.h"
#endif //not USER_STM32

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

#ifdef USER_STM32
  void Step_Direction(void);
  void Step_Pulse(void);
  void StepPulsePwmReset(void);
  void StepPulsePwm(void);
  void ISR_TIMER1_COMPA_vect(void);
  #ifdef USER_STEP_PWM
  #else
    void ISR_TIMER0_OVF_vect(void);
    #ifdef STEP_PULSE_DELAY
      void ISR_TIMER0_COMPA_vect(void);
    #endif //STEP_PULSE_DELAY
  #endif
#endif //USER_STM32

// Initialize and setup the stepper motor subsystem
void stepper_init();
#ifdef USER_STM32
  void ComputationStepDelay(void);
#endif
// Enable steppers, but cycle does not start unless called by motion control or realtime command.
void st_wake_up();

// Immediately disables steppers
void st_go_idle();

// Generate the step and direction port invert masks.
void st_generate_step_dir_invert_masks();

// Reset the stepper subsystem variables
void st_reset();

// Changes the run state of the step segment buffer to execute the special parking motion.
void st_parking_setup_buffer();

// Restores the step segment buffer to the normal run state after a parking motion.
void st_parking_restore_buffer();

// Reloads step segment buffer. Called continuously by realtime execution system.
void st_prep_buffer();

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters();

// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
#ifdef USER_DOUBLE
  double st_get_realtime_rate();
#else
  float st_get_realtime_rate();
#endif

#endif
