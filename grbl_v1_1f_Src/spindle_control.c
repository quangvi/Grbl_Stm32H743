/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2017 Sungeun K. Jeon for Gnea Research LLC
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

#ifdef USER_STM32
  #include "main.h"
  #include "Extern_Variables_functions.h"

  uint8_t PWM_is_enabled = 0;
#else //not USER_STM32
  #include "grbl.h"
#endif //USER_STM32


#ifdef VARIABLE_SPINDLE
  #ifdef USER_DOUBLE_INCIDENTAL
    static double pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
  #else
    static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
  #endif
#endif


void spindle_init()
{
  #ifdef USER_STM32
    #ifdef VARIABLE_SPINDLE
        #ifdef USER_OUTPUT_OPEN_DRAIN
          GPIO_InitTypeDef GPIO_InitStruct;
          /*Configure GPIO pins : SPINDLE_DIRECTION_Pin*/
          GPIO_InitStruct.Pin = SPINDLE_DIRECTION_Pin;
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;//GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
          HAL_GPIO_Init(SPINDLE_DIRECTION_GPIO_Port, &GPIO_InitStruct);
          /*Configure GPIO pins : SPINDLE_ENABLE_Pin*/
          GPIO_InitStruct.Pin = SPINDLE_ENABLE_Pin;
          HAL_GPIO_Init(SPINDLE_ENABLE_GPIO_Port, &GPIO_InitStruct);
          /**TIM4 GPIO Configuration
          PD15     ------> TIM4_CH4
          */
          GPIO_InitStruct.Pin = LD6_SPINDLE_PWM_Pin;
          GPIO_InitStruct.Mode =   GPIO_MODE_AF_OD;//GPIO_MODE_AF_PP;
          GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
          HAL_GPIO_Init(LD6_SPINDLE_PWM_GPIO_Port, &GPIO_InitStruct);
        #else //not USER_OUTPUT_OPEN_DRAIN
          //
        #endif //USER_OUTPUT_OPEN_DRAIN
        /*##- Configure the TIM peripheral #######################################*/
          /*##- Start the TIM11 Base generation for Delay_2  ####################*/
          RCC_ClkInitTypeDef    clkconfig;
          uint32_t              uwTimclock = 0;
          uint32_t              uwPrescalerValue = 0;
          uint32_t              pFLatency;
          /* Get clock configuration */
          HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

          /* Compute TIM clock */
          uwTimclock = 2*HAL_RCC_GetPCLK1Freq();

          /* Compute the prescaler value to have TIM counter clock equal to 10MHz */
          //(((SystemCoreClock / APB1_PRESCALE) * 2) / 10000000) - 1;;
          uwPrescalerValue = (uint32_t) ((uwTimclock / 10000000) - 1);

        //  /* Set the Auto-reload value */
        //  htim6.Instance->ARR = 0xFFFF;
        //  /* Set the Prescaler value */
        //  htim6.Instance->PSC = uwPrescalerValue;

//        htim13.Instance = TIM13;
        htim13.Init.Prescaler = uwPrescalerValue;
        htim13.Init.Period = SPINDLE_PWM_RANGE;

        if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
        {
          /* Starting Error */
          Error_Handler();
        }

        if(HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1) != HAL_OK)
        {
          /* Starting Error */
          Error_Handler();
        }
//        __HAL_TIM_MOE_ENABLE(&htim13);
        pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);
    #else //not VARIABLE_SPINDLE
      #ifdef USER_OUTPUT_OPEN_DRAIN
        GPIO_InitTypeDef GPIO_InitStruct;
        /*Configure GPIO pins : SPINDLE_DIRECTION_Pin*/
        GPIO_InitStruct.Pin = SPINDLE_DIRECTION_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;//GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(SPINDLE_DIRECTION_GPIO_Port, &GPIO_InitStruct);
        /*Configure GPIO pins : SPINDLE_ENABLE_Pin*/
        GPIO_InitStruct.Pin = SPINDLE_ENABLE_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;//GPIO_MODE_OUTPUT_PP;
        HAL_GPIO_Init(SPINDLE_ENABLE_GPIO_Port, &GPIO_InitStruct);
        /**TIM4 GPIO Configuration
        PD15     ------> TIM4_CH4
        */
        GPIO_InitStruct.Pin = LD6_SPINDLE_PWM_Pin;
        GPIO_InitStruct.Mode =   GPIO_MODE_INPUT;//GPIO_MODE_AF_PP;
  //        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(LD6_SPINDLE_PWM_GPIO_Port, &GPIO_InitStruct);
      #else //not USER_OUTPUT_OPEN_DRAIN
        //
      #endif //USER_OUTPUT_OPEN_DRAIN
    #endif //VARIABLE_SPINDLE
  #else //not USER_STM32
    #ifdef VARIABLE_SPINDLE

      // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
      // combined unless configured otherwise.
      SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
      SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
      SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
      #else
        SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
      #endif

      pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);

    #else

      // Configure no variable spindle and only enable pin.
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.

    #endif
  #endif //USER_STM32
  spindle_stop();
}


uint8_t spindle_get_state()
{
  #ifdef USER_STM32
    #ifdef VARIABLE_SPINDLE
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        // No spindle direction output pin.
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          if (bit_isfalse(SPINDLE_ENABLE_PIN_STM32, SPINDLE_ENABLE_Pin)) { return(SPINDLE_STATE_CW); }
        #else
          if (bit_istrue(SPINDLE_ENABLE_PIN_STM32, SPINDLE_ENABLE_Pin)) { return(SPINDLE_STATE_CW); }
        #endif
      #else
  //        if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
        if(PWM_is_enabled != 0)
        {
          if (bit_istrue(SPINDLE_DIRECTION_PIN_STM32, SPINDLE_DIRECTION_Pin)) { return(SPINDLE_STATE_CCW); }
          else { return(SPINDLE_STATE_CW); }
        }
      #endif
    #else //not VARIABLE_SPINDLE
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(SPINDLE_ENABLE_PIN_STM32, SPINDLE_ENABLE_Pin)) {
      #else
        if (bit_istrue(SPINDLE_ENABLE_PIN_STM32, SPINDLE_ENABLE_Pin)) {
      #endif
        if (bit_istrue(SPINDLE_DIRECTION_PIN_STM32, SPINDLE_DIRECTION_Pin)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      }
    #endif //VARIABLE_SPINDLE
  #else //not USER_STM32
    #ifdef VARIABLE_SPINDLE
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        // No spindle direction output pin.
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
        #else
          if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) { return(SPINDLE_STATE_CW); }
        #endif
      #else
        if (SPINDLE_TCCRA_REGISTER & (1<<SPINDLE_COMB_BIT)) { // Check if PWM is enabled.
          if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
          else { return(SPINDLE_STATE_CW); }
        }
      #endif
    #else
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        if (bit_isfalse(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
      #else
        if (bit_istrue(SPINDLE_ENABLE_PORT,(1<<SPINDLE_ENABLE_BIT))) {
      #endif
        if (SPINDLE_DIRECTION_PORT & (1<<SPINDLE_DIRECTION_BIT)) { return(SPINDLE_STATE_CCW); }
        else { return(SPINDLE_STATE_CW); }
      }
    #endif
  #endif //USER_STM32
	return(SPINDLE_STATE_DISABLE);
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
#ifdef USER_STM32
  void spindle_stop()
  {
    #ifdef VARIABLE_SPINDLE
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
    PWM_is_enabled = 0;
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          //__HAL_GPIO_SET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);  // Set pin to high
          LL_GPIO_SetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
        #else
          //__HAL_GPIO_RESET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin); // Set pin to low
          LL_GPIO_ResetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
        #endif
      #endif
    #else //not VARIABLE_SPINDLE
      PWM_is_enabled = 0;
      #ifdef INVERT_SPINDLE_ENABLE_PIN
      //__HAL_GPIO_SET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);  // Set pin to high
      LL_GPIO_SetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
      #else
      //__HAL_GPIO_RESET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);; // Set pin to low
      LL_GPIO_ResetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
      #endif
    #endif //VARIABLE_SPINDLE
  }
#else //not USER_STM32
  void spindle_stop()
  {
    #ifdef VARIABLE_SPINDLE
      SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
        #else
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
        #endif
      #endif
    #else
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
      #endif
    #endif
  }
#endif //USER_STM32

#ifdef VARIABLE_SPINDLE
  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  #ifdef USER_STM32
    void spindle_set_speed(uint16_t pwm_value)
  #else
    void spindle_set_speed(uint8_t pwm_value)
  #endif
  {
    #ifdef USER_STM32
      uint16_t temp = (pwm_value*PWM_PER_100)/100;
      __MY_HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, temp);// Set PWM output level.
      #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
        if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
          spindle_stop();
        } else {
//          SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
          #ifdef INVERT_SPINDLE_ENABLE_PIN
            //__HAL_GPIO_RESET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
            LL_GPIO_ResetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
          #else
            //__HAL_GPIO_SET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
            LL_GPIO_SetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
          #endif
        }
      #else
        //not thing
      #endif
    #else //not USER_STM32
      SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
      #ifdef SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED
        if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
          spindle_stop();
        } else {
          SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
          #ifdef INVERT_SPINDLE_ENABLE_PIN
            SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
          #else
            SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
          #endif
        }
      #else
        if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
          SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
        } else {
          SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
        }
      #endif
    #endif //USER_STM32
    }

  #ifdef ENABLE_PIECEWISE_LINEAR_SPINDLE
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    #ifdef USER_DOUBLE_INCIDENTAL
      uint8_t spindle_compute_pwm_value(double rpm) // 328p PWM register is 8-bit.
    #else
      uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
    #endif
    {
      uint8_t pwm_value;
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= RPM_MAX)) {
        rpm = RPM_MAX;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= RPM_MIN) {
        if (rpm == 0.0) { // S0 disables spindle
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else {
          rpm = RPM_MIN;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else {
        // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
        #if (N_PIECES > 3)
          if (rpm > RPM_POINT34) {
            pwm_value = floor(RPM_LINE_A4*rpm - RPM_LINE_B4);
          } else 
        #endif
        #if (N_PIECES > 2)
          if (rpm > RPM_POINT23) {
            pwm_value = floor(RPM_LINE_A3*rpm - RPM_LINE_B3);
          } else 
        #endif
        #if (N_PIECES > 1)
          if (rpm > RPM_POINT12) {
            pwm_value = floor(RPM_LINE_A2*rpm - RPM_LINE_B2);
          } else 
        #endif
        {
          pwm_value = floor(RPM_LINE_A1*rpm - RPM_LINE_B1);
        }
      }
      sys.spindle_speed = rpm;
      return(pwm_value);
    }
    
  #else
    // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
    #ifdef USER_STM32
      #ifdef USER_DOUBLE_INCIDENTAL
        uint16_t spindle_compute_pwm_value(double rpm) // 328p PWM register is 16-bit.
      #else
        uint16_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 16-bit.
      #endif
      {
        uint16_t pwm_value;
    #else
      uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
      {
        uint8_t pwm_value;
    #endif
      rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_speed = settings.rpm_max;
        pwm_value = SPINDLE_PWM_MAX_VALUE;
      } else if (rpm <= settings.rpm_min) {
        if (rpm == 0.0) { // S0 disables spindle
          sys.spindle_speed = 0.0;
          pwm_value = SPINDLE_PWM_OFF_VALUE;
        } else { // Set minimum PWM output
          sys.spindle_speed = settings.rpm_min;
          pwm_value = SPINDLE_PWM_MIN_VALUE;
        }
      } else { 
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_speed = rpm;
        pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
      }
      return(pwm_value);
    }
    
  #endif
#endif


// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  #ifdef USER_DOUBLE_INCIDENTAL
    void spindle_set_state(uint8_t state, double rpm);
  #else
    void spindle_set_state(uint8_t state, float rpm)
  #endif
#else
  void _spindle_set_state(uint8_t state)
#endif
{
  if (sys.abort) { return; } // Block during abort.
  if (state == SPINDLE_DISABLE) { // Halt or set spindle direction and rpm.
  
    #ifdef VARIABLE_SPINDLE
      sys.spindle_speed = 0.0;
    #endif
    spindle_stop();
  
  } else {

    #ifdef USER_STM32
      #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
        if (state == SPINDLE_ENABLE_CW) {
          //__HAL_GPIO_RESET_BITS(SPINDLE_DIRECTION_GPIO_Port, SPINDLE_DIRECTION_Pin);
          LL_GPIO_ResetOutputPin(SPINDLE_DIRECTION_GPIO_Port, SPINDLE_DIRECTION_Pin);
        } else {
          //__HAL_GPIO_SET_BITS(SPINDLE_DIRECTION_GPIO_Port, SPINDLE_DIRECTION_Pin);
          LL_GPIO_SetOutputPin(SPINDLE_DIRECTION_GPIO_Port, SPINDLE_DIRECTION_Pin);
        }
      #endif
      #ifdef VARIABLE_SPINDLE
        #ifdef LASER_CONSTANT_POWER_PER_RATE
          // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
          if (settings.flags & BITFLAG_LASER_MODE) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
        #endif
        spindle_set_speed(spindle_compute_pwm_value(rpm));
        #endif
        #if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) || !defined(VARIABLE_SPINDLE)
          // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
          // if the spindle speed value is zero, as its ignored anyhow.
          PWM_is_enabled = 1;
          #ifdef INVERT_SPINDLE_ENABLE_PIN
            //__HAL_GPIO_RESET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin); // Set pin to low
            LL_GPIO_ResetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
          #else
            //__HAL_GPIO_SET_BITS(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);  // Set pin to high
            LL_GPIO_SetOutputPin(SPINDLE_ENABLE_GPIO_Port, SPINDLE_ENABLE_Pin);
          #endif
        #endif

    #else //not USER_STM32

      #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
        if (state == SPINDLE_ENABLE_CW) {
          SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
        } else {
          SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
        }
      #endif

      #ifdef VARIABLE_SPINDLE
        // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
        if (settings.flags & BITFLAG_LASER_MODE) {
          if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
        }
        spindle_set_speed(spindle_compute_pwm_value(rpm));
      #endif
      #if (defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && \
          !defined(SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED)) || !defined(VARIABLE_SPINDLE)
        // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
        // if the spindle speed value is zero, as its ignored anyhow.
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      #endif
    #endif //USER_STM32
  }
  
  sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  #ifdef USER_DOUBLE_INCIDENTAL
    void spindle_sync(uint8_t state, double rpm)
  #else
    void spindle_sync(uint8_t state, float rpm)
  #endif
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }
#endif
