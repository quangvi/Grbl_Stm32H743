/*
  cpu_map.h - CPU and pin mapping configuration file
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/* The cpu_map.h files serve as a central pin mapping selection file for different
   processor types or alternative pin layouts. This version of Grbl officially supports
   only the Arduino Mega328p. */


#ifndef cpu_map_h
#define cpu_map_h

#ifdef USER_STM32
  #include "Extern_Variables_functions.h"
#endif //not USER_STM32

#ifdef CPU_MAP_ATMEGA328P // (Arduino Uno) Officially supported by Grbl.

  // Define serial port pins and interrupt vectors.
  #define SERIAL_RX     USART_RX_vect
  #define SERIAL_UDRE   USART_UDRE_vect

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #define STEP_DDR        DDRD
  #define STEP_PORT       PORTD
  #define X_STEP_BIT      2  // Uno Digital Pin 2
  #define Y_STEP_BIT      3  // Uno Digital Pin 3
  #define Z_STEP_BIT      4  // Uno Digital Pin 4
  #define STEP_MASK       ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #define DIRECTION_DDR     DDRD
  #define DIRECTION_PORT    PORTD
  #define X_DIRECTION_BIT   5  // Uno Digital Pin 5
  #define Y_DIRECTION_BIT   6  // Uno Digital Pin 6
  #define Z_DIRECTION_BIT   7  // Uno Digital Pin 7
  #define DIRECTION_MASK    ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

  // Define stepper driver enable/disable output pin.
  #define STEPPERS_DISABLE_DDR    DDRB
  #define STEPPERS_DISABLE_PORT   PORTB
  #define STEPPERS_DISABLE_BIT    0  // Uno Digital Pin 8
  #define STEPPERS_DISABLE_MASK   (1<<STEPPERS_DISABLE_BIT)

  // Define homing/hard limit switch input pins and limit interrupt vectors.
  // NOTE: All limit bit pins must be on the same port, but not on a port with other input pins (CONTROL).
  #define LIMIT_DDR        DDRB
  #define LIMIT_PIN        PINB
  #define LIMIT_PORT       PORTB
  #define X_LIMIT_BIT      1  // Uno Digital Pin 9
  #define Y_LIMIT_BIT      2  // Uno Digital Pin 10
  #ifdef VARIABLE_SPINDLE // Z Limit pin and spindle enabled swapped to access hardware PWM on Pin 11.
    #define Z_LIMIT_BIT     4 // Uno Digital Pin 12
  #else
    #define Z_LIMIT_BIT    3  // Uno Digital Pin 11
  #endif
  #define LIMIT_MASK       ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
  #define LIMIT_INT        PCIE0  // Pin change interrupt enable pin
  #define LIMIT_INT_vect   PCINT0_vect
  #define LIMIT_PCMSK      PCMSK0 // Pin change interrupt register

  // Define spindle enable and spindle direction output pins.
  #define SPINDLE_ENABLE_DDR    DDRB
  #define SPINDLE_ENABLE_PORT   PORTB
  // Z Limit pin and spindle PWM/enable pin swapped to access hardware PWM on Pin 11.
  #ifdef VARIABLE_SPINDLE
    #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      // If enabled, spindle direction pin now used as spindle enable, while PWM remains on D11.
      #define SPINDLE_ENABLE_BIT    5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
    #else
      #define SPINDLE_ENABLE_BIT    3  // Uno Digital Pin 11
    #endif
  #else
    #define SPINDLE_ENABLE_BIT    4  // Uno Digital Pin 12
  #endif
  #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
    #define SPINDLE_DIRECTION_DDR   DDRB
    #define SPINDLE_DIRECTION_PORT  PORTB
    #define SPINDLE_DIRECTION_BIT   5  // Uno Digital Pin 13 (NOTE: D13 can't be pulled-high input due to LED.)
  #endif

  // Define flood and mist coolant enable output pins.
  #define COOLANT_FLOOD_DDR   DDRC
  #define COOLANT_FLOOD_PORT  PORTC
  #define COOLANT_FLOOD_BIT   3  // Uno Analog Pin 3
  #define COOLANT_MIST_DDR   DDRC
  #define COOLANT_MIST_PORT  PORTC
  #define COOLANT_MIST_BIT   4  // Uno Analog Pin 4

  // Define user-control controls (cycle start, reset, feed hold) input pins.
  // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
  #define CONTROL_DDR       DDRC
  #define CONTROL_PIN       PINC
  #define CONTROL_PORT      PORTC
  #define CONTROL_RESET_BIT         0  // Uno Analog Pin 0
  #define CONTROL_FEED_HOLD_BIT     1  // Uno Analog Pin 1
  #define CONTROL_CYCLE_START_BIT   2  // Uno Analog Pin 2
  #define CONTROL_SAFETY_DOOR_BIT   1  // Uno Analog Pin 1 NOTE: Safety door is shared with feed hold. Enabled by config define.
  #define CONTROL_INT       PCIE1  // Pin change interrupt enable pin
  #define CONTROL_INT_vect  PCINT1_vect
  #define CONTROL_PCMSK     PCMSK1 // Pin change interrupt register
  #define CONTROL_MASK      ((1<<CONTROL_RESET_BIT)|(1<<CONTROL_FEED_HOLD_BIT)|(1<<CONTROL_CYCLE_START_BIT)|(1<<CONTROL_SAFETY_DOOR_BIT))
  #define CONTROL_INVERT_MASK   CONTROL_MASK // May be re-defined to only invert certain control pins.

  // Define probe switch input pin.
  #define PROBE_DDR       DDRC
  #define PROBE_PIN       PINC
  #define PROBE_PORT      PORTC
  #define PROBE_BIT       5  // Uno Analog Pin 5
  #define PROBE_MASK      (1<<PROBE_BIT)

  // Variable spindle configuration below. Do not change unless you know what you are doing.
  // NOTE: Only used when variable spindle is enabled.
  #define SPINDLE_PWM_MAX_VALUE     255 // Don't change. 328p fast PWM mode fixes top value as 255.
  #ifndef SPINDLE_PWM_MIN_VALUE
    #define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
  #endif
  #define SPINDLE_PWM_OFF_VALUE     0
  #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)
  #define SPINDLE_TCCRA_REGISTER    TCCR2A
  #define SPINDLE_TCCRB_REGISTER    TCCR2B
  #define SPINDLE_OCR_REGISTER      OCR2A
  #define SPINDLE_COMB_BIT          COM2A1

  // Prescaled, 8-bit Fast PWM mode.
  #define SPINDLE_TCCRA_INIT_MASK   ((1<<WGM20) | (1<<WGM21))  // Configures fast PWM mode.
  // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS20)               // Disable prescaler -> 62.5kHz
  // #define SPINDLE_TCCRB_INIT_MASK   (1<<CS21)               // 1/8 prescaler -> 7.8kHz (Used in v0.9)
  // #define SPINDLE_TCCRB_INIT_MASK   ((1<<CS21) | (1<<CS20)) // 1/32 prescaler -> 1.96kHz
  #define SPINDLE_TCCRB_INIT_MASK      (1<<CS22)               // 1/64 prescaler -> 0.98kHz (J-tech laser)

  // NOTE: On the 328p, these must be the same as the SPINDLE_ENABLE settings.
  #define SPINDLE_PWM_DDR    DDRB
  #define SPINDLE_PWM_PORT  PORTB
  #define SPINDLE_PWM_BIT    3    // Uno Digital Pin 11

#endif


#ifdef CPU_MAP_CUSTOM_PROC
  // For a custom pin map or different processor, copy and edit one of the available cpu
  // map files and modify it to your needs. Make sure the defined name is also changed in
  // the config.h file.

  // Define step pulse output pins. NOTE: All step bit pins must be on the same port.
  #ifdef STM32H743xx
    #ifdef USER_PULSE_NOT_EQUAL_DIR
      #define STEP_GPIO_Port GPIOE
    #else
      #define STEP_GPIO_Port GPIOF
    #endif
  #else
    #ifdef USER_PULSE_NOT_EQUAL_DIR
      #define STEP_GPIO_Port GPIOE
    #else
      #define STEP_GPIO_Port GPIOE
    #endif
  #endif
  #define X_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x01U//bit(0)
  #define Y_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x02U//bit(1)
  #define Z_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x04U//bit(2)

  #ifdef USER_MORE_AXIS
    #define A_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x08U//bit(3)
    #ifdef USER_6_AXIS
      #define B_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x10U//bit(4)
      #define C_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x20U//bit(5)
      #ifdef USER_8_AXIS
        #define D_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x40U//bit(6)
        #define E_STEP_BIT_SIMULATE_INDEX    (uint8_t)0x80U//bit(7)
      #endif
    #endif
    #ifdef USER_6_AXIS
      #ifdef USER_8_AXIS
        #define STEP_MASK_PORT 0xFF00
//        #define STEP_MASK_SIMULATE 0xFF //((1<<X_STEP_BIT_SIMULATE)|(1<<Y_STEP_BIT_SIMULATE)|(1<<Z_STEP_BIT_SIMULATE)|(1<<A_STEP_BIT_SIMULATE)) // All step bits
        #define STEP_MASK_SIMULATE_INV 0x00
      #else
        #define STEP_MASK_PORT 0xFF00
        #define STEP_MASK_SIMULATE 0x3F //((1<<X_STEP_BIT_SIMULATE)|(1<<Y_STEP_BIT_SIMULATE)|(1<<Z_STEP_BIT_SIMULATE)|(1<<A_STEP_BIT_SIMULATE)) // All step bits
        #define STEP_MASK_SIMULATE_INV 0xC0
      #endif
    #else
      #define STEP_MASK_PORT 0xFF00
      #define STEP_MASK_SIMULATE 0x0F
      #define STEP_MASK_SIMULATE_INV 0xF0
    #endif
  #else //not USER_MORE_AXIS
    #define STEP_MASK_PORT 0xFF00
    #define STEP_MASK_SIMULATE 0x07 //((1<<X_STEP_BIT_SIMULATE)|(1<<Y_STEP_BIT_SIMULATE)|(1<<Z_STEP_BIT_SIMULATE)) // All step bits
    #define STEP_MASK_SIMULATE_INV 0xF8
  #endif //USER_MORE_AXIS

  // Define step direction output pins. NOTE: All direction pins must be on the same port.
  #ifdef STM32H743xx
    #ifdef USER_PULSE_NOT_EQUAL_DIR
      #define DIRECTION_GPIO_Port GPIOD
    #else
      #define DIRECTION_GPIO_Port GPIOF
    #endif
  #else
    #ifdef USER_PULSE_NOT_EQUAL_DIR
      #define DIRECTION_GPIO_Port GPIOD
    #else
      #define DIRECTION_GPIO_Port GPIOE
    #endif
  #endif
  #define X_DIRECTION_BIT_SIMULATE_INDEX   (uint8_t)0x01U//bit(0)
  #define Y_DIRECTION_BIT_SIMULATE_INDEX   (uint8_t)0x02U//bit(1)
  #define Z_DIRECTION_BIT_SIMULATE_INDEX   (uint8_t)0x04U//bit(2)

  #ifdef USER_MORE_AXIS
    #define A_DIRECTION_BIT_SIMULATE_INDEX    (uint8_t)0x08U//bit(3)
    #ifdef USER_6_AXIS
      #define B_DIRECTION_BIT_SIMULATE_INDEX    (uint8_t)0x10U//bit(4)
      #define C_DIRECTION_BIT_SIMULATE_INDEX    (uint8_t)0x20U//bit(5)
      #ifdef USER_8_AXIS
        #define D_DIRECTION_BIT_SIMULATE_INDEX    (uint8_t)0x40U//bit(6)
        #define E_DIRECTION_BIT_SIMULATE_INDEX    (uint8_t)0x80U//bit(7)
      #endif
    #endif
    #ifdef USER_6_AXIS
      #ifdef USER_8_AXIS
        #define DIRECTION_MASK_PORT 0x00FF
//        #define DIRECTION_MASK_SIMULATE 0xFF //((1<<X_DIRECTION_BIT_SIMULATE)|(1<<Y_DIRECTION_BIT_SIMULATE)|(1<<Z_DIRECTION_BIT_SIMULATE)|(1<<A_DIRECTION_BIT_SIMULATE)) // All direction bits
        #define DIRECTION_MASK_SIMULATE_INV 0x00
      #else
        #define DIRECTION_MASK_PORT 0x00FF
        #define DIRECTION_MASK_SIMULATE 0x3F //((1<<X_DIRECTION_BIT_SIMULATE)|(1<<Y_DIRECTION_BIT_SIMULATE)|(1<<Z_DIRECTION_BIT_SIMULATE)|(1<<A_DIRECTION_BIT_SIMULATE)) // All direction bits
        #define DIRECTION_MASK_SIMULATE_INV 0xC0
      #endif
    #else
      #define DIRECTION_MASK_PORT 0x00FF
      #define DIRECTION_MASK_SIMULATE 0x0F //((1<<X_DIRECTION_BIT_SIMULATE)|(1<<Y_DIRECTION_BIT_SIMULATE)|(1<<Z_DIRECTION_BIT_SIMULATE)|(1<<A_DIRECTION_BIT_SIMULATE)) // All direction bits
      #define DIRECTION_MASK_SIMULATE_INV 0xF0
    #endif

  #else //not USER_MORE_AXIS
    #define DIRECTION_MASK_PORT 0x00FF
    #define DIRECTION_MASK_SIMULATE 0x07//((1<<X_DIRECTION_BIT_SIMULATE)|(1<<Y_DIRECTION_BIT_SIMULATE)|(1<<Z_DIRECTION_BIT_SIMULATE)) // All direction bits
    #define DIRECTION_MASK_SIMULATE_INV 0xF8
  #endif //USER_MORE_AXIS

  // Variable spindle configuration below. Do not change unless you know what you are doing.
  // NOTE: Only used when variable spindle is enabled.
  #define SPINDLE_PWM_MAX_VALUE     1000 // Don't change. 328p fast PWM mode fixes top value as 255.
  #ifdef SPINDLE_MINIMUM_PWM
    #define SPINDLE_PWM_MIN_VALUE   SPINDLE_MINIMUM_PWM
  #else
    #define SPINDLE_PWM_MIN_VALUE   1   // Must be greater than zero.
  #endif
  #define SPINDLE_PWM_OFF_VALUE     0
  #define SPINDLE_PWM_RANGE         (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)

  #define SPINDLE_ENABLE_PIN_STM32 SPINDLE_ENABLE_GPIO_Port->IDR
  #define SPINDLE_DIRECTION_PIN_STM32 SPINDLE_DIRECTION_GPIO_Port->IDR
  // Define homing/hard limit switch input pins and limit interrupt vectors.
  // NOTE: All limit bit pins must be on the same port
  #ifdef STM32H743xx
      #define LIMIT_PORT_STM32  GPIOD
      #define LIMIT_PIN_STM32   GPIOD->IDR
  #else
      #define LIMIT_PORT_STM32  GPIOB
      #define LIMIT_PIN_STM32   GPIOB->IDR
    #endif
    // Define control pin index for Grbl internal use. Pin maps may change, but these values don't.
    #define X_LIMIT_PIN_INDEX       (uint8_t)0x01U//bit(0)
    #define Y_LIMIT_PIN_INDEX       (uint8_t)0x02U//bit(1)
    #define Z_LIMIT_PIN_INDEX       (uint8_t)0x04U//bit(2)

    #ifdef USER_MORE_AXIS
      #define A_LIMIT_PIN_INDEX       (uint8_t)0x08U//bit(3)
      #ifdef USER_6_AXIS
        #define B_LIMIT_PIN_INDEX    (uint8_t)0x10U//bit(4)
        #define C_LIMIT_PIN_INDEX    (uint8_t)0x20U//bit(5)
        #ifdef USER_8_AXIS
          #define D_LIMIT_PIN_INDEX    (uint8_t)0x40U//bit(6)
          #define E_LIMIT_PIN_INDEX    (uint8_t)0x80U//bit(7)
        #endif
      #endif
      #ifdef USER_6_AXIS
        #ifdef USER_8_AXIS
          //#define LIMIT_MASK_STM32  (uint8_t)0xFFU // All limit bits
          #define EXTI_LINE_LIMIT_MASK
        #else
          #define LIMIT_MASK_STM32  (uint8_t)0x3FU // All limit bits
        #endif
      #else
        #define LIMIT_MASK_STM32  ((X_LIMIT_Pin)|(Y_LIMIT_Pin)|(Z_LIMIT_Pin)|(A_LIMIT_Pin)) // All limit bits
      #endif
      #define N_LIMIT_PIN_INDEX  (uint8_t)0x0FU//((X_LIMIT_PIN_INDEX)|(Y_LIMIT_PIN_INDEX)|(Z_LIMIT_PIN_INDEX)|(A_LIMIT_PIN_INDEX))
    #else //not USER_MORE_AXIS
      #define N_LIMIT_PIN_INDEX  (uint8_t)0x07U//((X_LIMIT_PIN_INDEX)|(Y_LIMIT_PIN_INDEX)|(Z_LIMIT_PIN_INDEX))
    #endif //USER_MORE_AXIS

    // Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
    // NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
    #ifdef STM32H743xx
      #define CONTROL_PIN_STM32        GPIOC->IDR
      #define CONTROL_PORT_STM32       GPIOC
  #else
    #define CONTROL_PIN_STM32        GPIOA->IDR
      #define CONTROL_PORT_STM32       GPIOA
    #endif

//    #define CONTROL_MASK_STM32      ((1<<CONTROL_RESET_BIT_STM32)|(1<<CONTROL_FEED_HOLD_BIT_STM32)|(1<<CONTROL_CYCLE_START_BIT_STM32)|(1<<CONTROL_SAFETY_DOOR_BIT_STM32))
    #define CONTROL_MASK_STM32 B1_RESET_GRBL_Pin|FEED_HOLD_Pin|CYCLE_START_Pin|SAFETY_DOOR_Pin

    // Define flood and mist coolant enable output pins.

    #define COOLANT_FLOOD_PIN_STM32     COOLANT_FLOOD_GPIO_Port->IDR
    #define COOLANT_MIST_PIN_STM32     COOLANT_MIST_GPIO_Port->IDR

    // Define probe switch input pin.
    #define PROBE_PIN_STM32     PROBE_GPIO_Port->IDR
    #define PROBE_BIT_SIMULATE       4  // MEGA2560 Analog Pin 15
    #define PROBE_MASK_SIMULATE      (1<<PROBE_BIT_SIMULATE)
/**
  * @brief  Clears and Sets the GPIO pin.
  * @param  __SET_GPIO__: specifies the GPIO to clear.
  * @param  __GPIO_PIN__: This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
  #ifdef STM32H743xx
    //#define __HAL_GPIO_SET_BITS(__GPIO_PORT__, __GPIO_PIN__) (__GPIO_PORT__->BSRR = __GPIO_PIN__)
    //#define __HAL_GPIO_RESET_BITS(__GPIO_PORT__, __GPIO_PIN__) (__GPIO_PORT__->BSRR = (uint32_t)__GPIO_PIN__<< 16)
    //#define __HAL_GPIO_TOGGLE_BITS(__GPIO_PORT__, __GPIO_PIN__) (__GPIO_PORT__->ODR ^= __GPIO_PIN__)
    //Not for TIM_CHANNEL_5 and 6
    #define __MY_HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
    (*(__IO uint32_t *)(&((__HANDLE__)->Instance->CCR1) + ((__CHANNEL__) >> 2U)) = (__COMPARE__))
//    #define __HAL_TIM_SET_COMPARE1(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR1 = __COMPARE__)
//    #define __HAL_TIM_SET_COMPARE2(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR2 = __COMPARE__)
//    #define __HAL_TIM_SET_COMPARE3(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR3 = __COMPARE__)
//    #define __HAL_TIM_SET_COMPARE4(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR4 = __COMPARE__)
    #define __HAL_TIM_SET_COMPARE5(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR5 = __COMPARE__)
    #define __HAL_TIM_SET_COMPARE6(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR6 = __COMPARE__)
    #define __MY_HAL_TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__) \
      (__HANDLE__)->Instance->ARR = (__AUTORELOAD__);
  #else
    #define __HAL_GPIO_SET_BITS(__GPIO_PORT__, __GPIO_PIN__) (__GPIO_PORT__->BSRR = __GPIO_PIN__)
    #define __HAL_GPIO_RESET_BITS(__GPIO_PORT__, __GPIO_PIN__) (__GPIO_PORT__->BSRR = (uint32_t)__GPIO_PIN__<<16U)
    #define __MY_HAL_TIM_DISABLE(__HANDLE__) (__HANDLE__)->Instance->CR1 &= ~(TIM_CR1_CEN)
    #define __MY_HAL_TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__) ((__HANDLE__)->Instance->ARR = (__AUTORELOAD__))
    #define __HAL_TIM_SET_COMPARE1(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR1 = (__COMPARE__))
    #define __HAL_TIM_SET_COMPARE2(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR2 = (__COMPARE__))
    #define __HAL_TIM_SET_COMPARE3(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR3 = (__COMPARE__))
    #define __HAL_TIM_SET_COMPARE4(__HANDLE__, __COMPARE__) ((__HANDLE__)->Instance->CCR4 = (__COMPARE__))
  #endif
#endif

#endif
