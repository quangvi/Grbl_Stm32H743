/*
  nuts_bolts.h - Header file for shared definitions, variables, and functions
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

#ifndef nuts_bolts_h
#define nuts_bolts_h

#ifdef USER_STM32
  #include "Extern_Variables_functions.h"
#endif //not USER_STM32

#define false 0
#define true 1

#define SOME_LARGE_VALUE 1.0E+38

// Axis array index values. Must start with 0 and be continuous.
#ifdef USER_MORE_AXIS
  #ifdef USER_6_AXIS
    #define A_AXIS 3
    #define B_AXIS 4
    #define C_AXIS 5
    #ifdef USER_8_AXIS
      #define N_AXIS 8 // Number of axes
      /* N_AXIS - 2 = 8 -2 = 4*/
      #define N_AXIS_2 6
      #define D_AXIS 6
      #define E_AXIS 7
    #else
      #define N_AXIS 6 // Number of axes
      /* N_AXIS - 2 = 6 -2 = 4*/
      #define N_AXIS_2 4
    #endif
  #else
    #define N_AXIS 4 // Number of axes
    #define A_AXIS 3
   /* N_AXIS - 2 = 4 -2 = 2*/
    #define N_AXIS_2 2
  #endif
#else //not USER_MORE_AXIS
  #define N_AXIS 3 // Number of axes
#endif //USER_MORE_AXIS
#define X_AXIS 0 // Axis indexing value.
#define Y_AXIS 1
#define Z_AXIS 2
// #define A_AXIS 3

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#ifdef COREXY
 #define A_MOTOR X_AXIS // Must be X_AXIS
 #define B_MOTOR Y_AXIS // Must be Y_AXIS
#endif

// Conversions
#define MM_PER_INCH (25.40)
#define INCH_PER_MM (0.0393701)
#ifdef USER_STM32
  #define TICKS_PER_SECOND (F_CPU)
  #define TICKS_PER_SECOND_60 (F_CPU_60)
//  #define TICKS_PER_MICROSECOND ((double)F_CPU/1000000)
#else
  #define TICKS_PER_MICROSECOND (F_CPU/1000000)
#endif

#define DELAY_MODE_DWELL       0
#define DELAY_MODE_SYS_SUSPEND 1

// Useful macros
#define clear_vector(a) memset(a, 0, sizeof(a))
#ifdef USER_DOUBLE
  #define clear_vector_float(a) memset(a, 0.0, sizeof(double)*N_AXIS)
#else
  #define clear_vector_float(a) memset(a, 0.0, sizeof(float)*N_AXIS)
#endif
// #define clear_vector_long(a) memset(a, 0.0, sizeof(long)*N_AXIS)
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
#ifdef USER_DOUBLE
  #define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(double)*N_AXIS))
#else
  #define isequal_position_vector(a,b) !(memcmp(a, b, sizeof(float)*N_AXIS))
#endif
// Bit field and masking macros
#define bit(n) (1 << n)
#define bit_true(x,mask) (x) |= (mask)
#define bit_false(x,mask) (x) &= ~(mask)
#define bit_istrue(x,mask) ((x & mask) != 0)
#define bit_isfalse(x,mask) ((x & mask) == 0)

// Read a floating point value from a string. Line points to the input buffer, char_counter
// is the indexer pointing to the current character of the line, while float_ptr is
// a pointer to the result variable. Returns true when it succeeds
#ifdef USER_DOUBLE
  uint8_t read_float(char *line, uint8_t *char_counter, double *float_ptr);
#else
  uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr);
#endif

// Non-blocking delay function used for general operation and suspend features.
#ifdef USER_DOUBLE_INCIDENTAL
  void delay_sec(double seconds, uint8_t mode);
#else
  void delay_sec(float seconds, uint8_t mode);
#endif

#ifdef USER_STM32

#else //not USER_STM32
  // Delays variable-defined milliseconds. Compiler compatibility fix for _delay_ms().
  void delay_ms(uint16_t ms);

  // Delays variable-defined microseconds. Compiler compatibility fix for _delay_us().
  void delay_us(uint32_t us);
#endif //USER_STM32

// Computes hypotenuse, avoiding avr-gcc's bloated version and the extra error checking.
#ifdef USER_DOUBLE
  double hypot_f(double x, double y);
  double convert_delta_vector_to_unit_vector(double *vector);
  double limit_value_by_axis_maximum(double *max_value, double *unit_vec);
#else
  float hypot_f(float x, float y);
  float convert_delta_vector_to_unit_vector(float *vector);
  float limit_value_by_axis_maximum(float *max_value, float *unit_vec);
#endif

#endif
