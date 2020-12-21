/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifdef USER_STM32
  #include "main.h"
  #include "Extern_Variables_functions.h"
  uint8_t ProbePinSimulate;
#else //not USER_STM32
  #include "grbl.h"
#endif //USER_STM32


// Inverts the probe pin state depending on user settings and probing cycle mode.
uint8_t probe_invert_mask;


// Probe pin initialization routine.
void probe_init()
{
  #ifdef USER_STM32
    GPIO_InitTypeDef GPIO_InitStruct;
    /*Configure GPIO pin : PROBE_Pin */
    GPIO_InitStruct.Pin = PROBE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    #ifdef DISABLE_PROBE_PIN_PULL_UP
      GPIO_InitStruct.Pull = GPIO_NOPULL;
    #else
      GPIO_InitStruct.Pull = GPIO_PULLUP;
    #endif
    HAL_GPIO_Init(PROBE_GPIO_Port, &GPIO_InitStruct);
  #else //not USER_STM32
    PROBE_DDR &= ~(PROBE_MASK); // Configure as input pins
    #ifdef DISABLE_PROBE_PIN_PULL_UP
      PROBE_PORT &= ~(PROBE_MASK); // Normal low operation. Requires external pull-down.
    #else
      PROBE_PORT |= PROBE_MASK;    // Enable internal pull-up resistors. Normal high operation.
    #endif
  #endif //USER_STM32
    probe_configure_invert_mask(false); // Initialize invert mask.
}


// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
#ifdef USER_STM32
  void probe_configure_invert_mask(uint8_t is_probe_away)
  {
    probe_invert_mask = 0; // Initialize as zero.
    if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK_SIMULATE; }
    if (is_probe_away) { probe_invert_mask ^= PROBE_MASK_SIMULATE; }
  }
#else //not USER_STM32
  void probe_configure_invert_mask(uint8_t is_probe_away)
  {
    probe_invert_mask = 0; // Initialize as zero.
    if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
    if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
  }
#endif //USER_STM32

// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
#ifdef USER_STM32
  uint8_t probe_get_state()
  {
//    ProbePinSimulate = 0;
//    if(bit_istrue(PROBE_PIN_STM32 , PROBE_Pin)){
//      ProbePinSimulate |= PROBE_MASK_SIMULATE;
//    }
//    else{
//      ProbePinSimulate &= ~ PROBE_MASK_SIMULATE;
//    }
    if(bit_istrue(PROBE_PIN_STM32 , PROBE_Pin)){
      ProbePinSimulate = PROBE_MASK_SIMULATE;
    }
    else{
      ProbePinSimulate = 0;
    }
    return(ProbePinSimulate ^ probe_invert_mask);
  }
#else //not USER_STM32
  uint8_t probe_get_state() { return((PROBE_PIN & PROBE_MASK) ^ probe_invert_mask); }
#endif //USER_STM32

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor()
{
  if (probe_get_state()) {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}
