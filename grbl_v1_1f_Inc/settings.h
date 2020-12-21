/*
  settings.h - eeprom configuration handling
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

#ifndef settings_h
#define settings_h

#include "grbl.h"


// Version of the EEPROM data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 10  // NOTE: Check settings_reset() when moving to next version.

// Define bit flag masks for the boolean settings in settings.flag.
#define BITFLAG_REPORT_INCHES      bit(0)
#define BITFLAG_LASER_MODE         bit(1)
#define BITFLAG_INVERT_ST_ENABLE   bit(2)
#define BITFLAG_HARD_LIMIT_ENABLE  bit(3)
#define BITFLAG_HOMING_ENABLE      bit(4)
#define BITFLAG_SOFT_LIMIT_ENABLE  bit(5)
#define BITFLAG_INVERT_LIMIT_PINS  bit(6)
#define BITFLAG_INVERT_PROBE_PIN   bit(7)

// Define status reporting boolean enable bit flags in settings.status_report_mask
#ifdef USE_CLASSIC_GRBL_INTERFACE
  #define BITFLAG_RT_STATUS_MACHINE_POSITION  bit(0)
  #define BITFLAG_RT_STATUS_WORK_POSITION     bit(1)
  #define BITFLAG_RT_STATUS_PLANNER_BUFFER    bit(2)
  #define BITFLAG_RT_STATUS_SERIAL_RX         bit(3)
  #define BITFLAG_RT_STATUS_LIMIT_PINS        bit(4)
  #define BITFLAG_RT_STATUS_PROBE_PIN         bit(5)
  #define BITFLAG_RT_STATUS_CONTROL_PINS      bit(6)
  #define BITFLAG_RT_STATUS_OVERRIDES         bit(7)
#else
  #define BITFLAG_RT_STATUS_POSITION_TYPE     bit(0)
  #define BITFLAG_RT_STATUS_BUFFER_STATE      bit(1)
#endif

// Define settings restore bitflags.
#define SETTINGS_RESTORE_DEFAULTS bit(0)
#define SETTINGS_RESTORE_PARAMETERS bit(1)
#define SETTINGS_RESTORE_STARTUP_LINES bit(2)
#define SETTINGS_RESTORE_BUILD_INFO bit(3)
#ifndef SETTINGS_RESTORE_ALL
  #define SETTINGS_RESTORE_ALL 0xFF // All bitflags
#endif

// Define EEPROM memory address location values for Grbl settings and parameters
// NOTE: The Atmega328p has 1KB EEPROM. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_STARTUP_BLOCK  768U
#define EEPROM_ADDR_BUILD_INFO     942U

// Define EEPROM address indexing for coordinate parameters
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#define SETTING_INDEX_NCOORD N_COORDINATE_SYSTEM+1 // Total number of system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM    // Home position 1
#define SETTING_INDEX_G30    N_COORDINATE_SYSTEM+1  // Home position 2
// #define SETTING_INDEX_G92    N_COORDINATE_SYSTEM+2  // Coordinate offset (G92.2,G92.3 not supported)

// Define Grbl axis settings numbering scheme. Starts at START_VAL, every INCREMENT, over N_SETTINGS.
#ifdef USER_BACKLASH
  #ifdef USER_ACC_BACKLASH
    #define AXIS_N_SETTINGS          6
  #else
    #define AXIS_N_SETTINGS          5
  #endif
#else
  #define AXIS_N_SETTINGS          4
#endif
#define AXIS_SETTINGS_START_VAL  100 // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings

// Global persistent settings (Stored from byte EEPROM_ADDR_GLOBAL onwards)
typedef struct {
  // Axis settings
  #ifdef USER_DOUBLE
    double steps_per_mm[N_AXIS];
    double max_rate[N_AXIS];
    double acceleration[N_AXIS];
    double max_travel[N_AXIS];
    #ifdef USER_BACKLASH
      double backlash_mm[N_AXIS];
      #ifdef USER_ACC_BACKLASH
        double backlash_acceleration[N_AXIS];
      #endif
    #endif
  #else
    float steps_per_mm[N_AXIS];
    float max_rate[N_AXIS];
    float acceleration[N_AXIS];
    float max_travel[N_AXIS];
    #ifdef USER_BACKLASH
      float backlash_mm[N_AXIS];
      #ifdef USER_ACC_BACKLASH
        float backlash_acceleration[N_AXIS];
      #endif
    #endif
  #endif

  // Remaining Grbl settings
  #ifdef USER_STM32
    #ifdef USER_DOUBLE_INCIDENTAL
      double pulse_microseconds;
    #else
      float pulse_microseconds;
    #endif
  #else
    uint8_t pulse_microseconds;
  #endif

  uint8_t step_invert_mask;
  uint8_t dir_invert_mask;
  uint8_t stepper_idle_lock_time; // If max value 255, steppers do not disable.
  uint8_t status_report_mask; // Mask to indicate desired report data.
  #ifdef USER_DOUBLE
    double junction_deviation;
    double arc_tolerance;

    double rpm_max;
    double rpm_min;
  #else
    float junction_deviation;
    float arc_tolerance;

    float rpm_max;
    float rpm_min;
  #endif
  uint8_t flags;  // Contains default boolean settings

  uint8_t homing_dir_mask;
  #ifdef USER_DOUBLE
    double homing_feed_rate;
    double homing_seek_rate;
    uint16_t homing_debounce_delay;
    double homing_pulloff;
  #else
    float homing_feed_rate;
    float homing_seek_rate;
    uint16_t homing_debounce_delay;
    float homing_pulloff;
  #endif
} settings_t;
extern settings_t settings;

// Initialize the configuration subsystem (load settings from EEPROM)
void settings_init();

// Helper function to clear and restore EEPROM defaults
void settings_restore(uint8_t restore_flag);

// A helper method to set new settings from command line
#ifdef USER_DOUBLE
  uint8_t settings_store_global_setting(uint8_t parameter, double value);
#else
  uint8_t settings_store_global_setting(uint8_t parameter, float value);
#endif

// Stores the protocol line variable as a startup line in EEPROM
void settings_store_startup_line(uint8_t n, char *line);

// Reads an EEPROM startup line to the protocol line variable
uint8_t settings_read_startup_line(uint8_t n, char *line);

// Stores build info user-defined string
void settings_store_build_info(char *line);

// Reads build info user-defined string
uint8_t settings_read_build_info(char *line);

// Writes selected coordinate data to EEPROM
#ifdef USER_DOUBLE
  void settings_write_coord_data(uint8_t coord_select, double *coord_data);
#else
  void settings_write_coord_data(uint8_t coord_select, float *coord_data);
#endif
// Reads selected coordinate data from EEPROM
#ifdef USER_DOUBLE
  uint8_t settings_read_coord_data(uint8_t coord_select, double *coord_data);
#else
  uint8_t settings_read_coord_data(uint8_t coord_select, float *coord_data);
#endif

#ifdef USER_STM32
  // Returns the step pin mask according to Grbl's internal axis numbering
  uint8_t get_step_pin_mask(uint8_t axis_idx);

  // Returns the direction pin mask according to Grbl's internal axis numbering
  uint8_t get_direction_pin_mask(uint8_t axis_idx);

  // Returns the limit pin mask according to Grbl's internal axis numbering
  uint8_t get_limit_pin_mask(uint8_t axis_idx);
#else //USER_STM32
  // Returns the step pin mask according to Grbl's internal axis numbering
  uint8_t get_step_pin_mask(uint8_t i);

  // Returns the direction pin mask according to Grbl's internal axis numbering
  uint8_t get_direction_pin_mask(uint8_t i);

  // Returns the limit pin mask according to Grbl's internal axis numbering
  uint8_t get_limit_pin_mask(uint8_t i);
#endif //USER_STM32

#endif