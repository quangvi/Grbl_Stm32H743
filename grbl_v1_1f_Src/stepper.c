/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
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

#ifdef USER_STM32
  #include "main.h"
  #include "Extern_Variables_functions.h"
#else //not USER_STM32
  #include "grbl.h"
#endif //USER_STM32


// Some useful constants.
#ifdef USER_DOUBLE
  #define DT_SEGMENT ((double)1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment
#else
  #define DT_SEGMENT (1.0/(ACCELERATION_TICKS_PER_SECOND*60.0)) // min/segment
#endif
#define REQ_MM_INCREMENT_SCALAR 1.25
#define RAMP_ACCEL 0
#define RAMP_CRUISE 1
#define RAMP_DECEL 2
#define RAMP_DECEL_OVERRIDE 3

#define PREP_FLAG_RECALCULATE bit(0)
#define PREP_FLAG_HOLD_PARTIAL_BLOCK bit(1)
#define PREP_FLAG_PARKING bit(2)
#define PREP_FLAG_DECEL_OVERRIDE bit(3)

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  #define MAX_AMASS_LEVEL 3
  #define AMASS_LEVEL0 (F_CPU/16000)//: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
  #define AMASS_LEVEL1 (F_CPU/8000) // Over-drives ISR (x2). Defined as F_CPU/(Cutoff frequency in Hz)
  #define AMASS_LEVEL2 (F_CPU/4000) // Over-drives ISR (x4)
  #define AMASS_LEVEL3 (F_CPU/2000) // Over-drives ISR (x8)

  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
#endif


// Stores the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct {
  uint32_t steps[N_AXIS];
  uint32_t step_event_count;
  uint8_t direction_bits;
  #ifdef VARIABLE_SPINDLE
    uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
  #endif
} st_block_t;
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE-1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
typedef struct {
  #ifdef USER_DOUBLE
    uint32_t n_step;           // Number of step events to be executed for this segment
  #else
    uint16_t n_step;           // Number of step events to be executed for this segment
  #endif
  uint16_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
  uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
  #else
    #ifdef USER_STM32
      uint16_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
    #else
      uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
    #endif
  #endif
  #ifdef VARIABLE_SPINDLE
    uint8_t spindle_pwm;
  #endif
} segment_t;
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
  // Used by the bresenham line algorithm
  #ifdef USER_MORE_AXIS
    uint32_t counter[N_AXIS];        // Counter variables for the bresenham line tracer
  #else //not USER_MORE_AXIS
    uint32_t counter_x,        // Counter variables for the bresenham line tracer
             counter_y,
             counter_z;
  #endif //USER_MORE_AXIS
  #ifdef STEP_PULSE_DELAY
    uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
  #endif

  uint8_t execute_step;     // Flags step execution for each interrupt.

  #ifdef USER_STM32
    uint8_t dir_bits;
    uint16_t step_pulse_time;  // Step pulse reset time after step rise
    uint16_t step_pulse_time_half;  // Step pulse reset time before step rise
  #else //not USER_STM32
    uint8_t step_pulse_time;  // Step pulse reset time after step rise
  #endif //USER_STM32

  uint8_t step_outbits;         // The next stepping-bits to be output
  uint8_t dir_outbits;
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    uint32_t steps[N_AXIS];
  #endif

  #ifdef USER_DOUBLE
    uint32_t step_count;       // Steps remaining in line segment motion
  #else
    uint16_t step_count;       // Steps remaining in line segment motion
  #endif

  uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
  st_block_t *exec_block;   // Pointer to the block data for the segment being executed
  segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;

static stepper_t st;

// Step segment ring buffer indices
#ifdef USER_NON_VOLATILE
  static uint8_t segment_buffer_tail;
#else
  static volatile uint8_t segment_buffer_tail;
#endif
static uint8_t segment_buffer_head;
static uint8_t segment_next_head;

// Step and direction port invert masks.
static uint8_t step_port_invert_mask;
static uint8_t dir_port_invert_mask;

// Used to avoid ISR nesting of the "Stepper Driver Interrupt". Should never occur though.
#ifdef USER_NON_VOLATILE
  static uint8_t busy;
#else
  static volatile uint8_t busy;
#endif

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
  uint8_t st_block_index;  // Index of stepper common data block being prepped
  uint8_t recalculate_flag;
  #ifdef USER_DOUBLE
    double dt_remainder;
    double steps_remaining;
    double step_per_mm;
    double req_mm_increment;
  #else
    float dt_remainder;
    float steps_remaining;
    float step_per_mm;
    float req_mm_increment;
  #endif

  #ifdef PARKING_ENABLE
    uint8_t last_st_block_index;
    #ifdef USER_DOUBLE
      double last_steps_remaining;
      double last_step_per_mm;
      double last_dt_remainder;
    #else
      float last_steps_remaining;
      float last_step_per_mm;
      float last_dt_remainder;
    #endif
  #endif

  uint8_t ramp_type;      // Current segment ramp state

#ifdef USER_DOUBLE
  double mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  double current_speed;    // Current speed at the end of the segment buffer (mm/min)
  double maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  double exit_speed;       // Exit speed of executing block (mm/min)
  double accelerate_until; // Acceleration ramp end measured from end of block (mm)
  double decelerate_after; // Deceleration ramp start measured from end of block (mm)

  #ifdef VARIABLE_SPINDLE
    double inv_rate;    // Used by PWM laser mode to speed up segment calculations.
    uint8_t current_spindle_pwm;
  #endif
#else
  float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                          // NOTE: This value must coincide with a step(no mantissa) when converted.
  float current_speed;    // Current speed at the end of the segment buffer (mm/min)
  float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
  float exit_speed;       // Exit speed of executing block (mm/min)
  float accelerate_until; // Acceleration ramp end measured from end of block (mm)
  float decelerate_after; // Deceleration ramp start measured from end of block (mm)

  #ifdef VARIABLE_SPINDLE
    float inv_rate;    // Used by PWM laser mode to speed up segment calculations.
    uint8_t current_spindle_pwm; 
  #endif
#endif
} st_prep_t;
static st_prep_t prep;


/*    BLOCK VELOCITY PROFILE DEFINITION
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity

  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  +
                    +--------+ <- maximum_speed (= nominal_speed)          /|\
                   /          \                                           / | \
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                     current_speed -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)

  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters
  are shown and defined in the above illustration.
*/

inline void ComputationStepDelay(void)
{
  // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
  st.step_pulse_time = (uint16_t)((settings.pulse_microseconds*TICKS_PER_MICROSECOND_DELAY));
  st.step_pulse_time_half = (st.step_pulse_time/2) - 1 +(TICKS_PER_MICROSECOND_DELAY/8);//1/4
  #ifdef USER_STEP_PWM
    st.step_pulse_time = st.step_pulse_time + st.step_pulse_time_half;//3/4
    // Set delay between direction pin write and step command.
    __HAL_TIM_SET_PRESCALER(&htim1, 0);
    __MY_HAL_TIM_SET_AUTORELOAD(&htim1, st.step_pulse_time);//~239
    #ifdef USER_6_AXIS
      __HAL_TIM_SET_PRESCALER(&htim8, 0);
      __MY_HAL_TIM_SET_AUTORELOAD(&htim8, st.step_pulse_time);//~239
    #endif
    st.step_pulse_time += 1;//~240
  #else
    // Set delay between direction pin write and step command.
    st.step_pulse_time_half += (TICKS_PER_MICROSECOND_DELAY/8);
//    __HAL_TIM_SET_PRESCALER(&htim14, 0);
//    __MY_HAL_TIM_SET_AUTORELOAD(&htim14, st.step_pulse_time - 1);
    LL_TIM_SetPrescaler(TIM14, 0);
    LL_TIM_SetAutoReload(TIM14, st.step_pulse_time - 1);
  #endif
}
// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up()
{
  // Enable stepper drivers.
  #ifdef USER_STM32
    if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) {
      //__HAL_GPIO_SET_BITS(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);// Set pin to high
      LL_GPIO_SetOutputPin(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);
    }
    else {
      //__HAL_GPIO_RESET_BITS(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);// Set pin to low
      LL_GPIO_ResetOutputPin(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);
    }
  #else //not USER_STM32
    if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
    else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
  #endif //USER_STM32

  // Initialize stepper output bits to ensure first ISR call does not step.
  st.step_outbits = step_port_invert_mask;

  // Initialize step pulse timing from settings. Here to ensure updating after re-writing.
  #ifdef USER_STM32
    ComputationStepDelay();
    #ifdef USER_STEP_PWM
      StepPulsePwmReset();
    #endif
  #else //not USER_STM32
    #ifdef STEP_PULSE_DELAY
      // Set total step pulse time after direction pin set. Ad hoc computation from oscilloscope.
      st.step_pulse_time = -(((settings.pulse_microseconds+STEP_PULSE_DELAY-2)*TICKS_PER_MICROSECOND) >> 3);
      // Set delay between direction pin write and step command.
      OCR0A = -(((settings.pulse_microseconds)*TICKS_PER_MICROSECOND) >> 3);
    #else // Normal operation
      // Set step pulse time. Ad hoc computation from oscilloscope. Uses two's complement.
      st.step_pulse_time = -(((settings.pulse_microseconds-2)*TICKS_PER_MICROSECOND) >> 3);
    #endif
  #endif //USER_STM32

  // Enable Stepper Driver Interrupt
  #ifdef USER_STM32
//    stepper_init();
    /* Clear the IT pending Bit */
    //__HAL_TIM_CLEAR_IT(&htim12, TIM_IT_UPDATE);
    LL_TIM_ClearFlag_UPDATE(TIM12);

    /* Enable the TIM Update interrupt */
    //__HAL_TIM_ENABLE_IT(&htim12, TIM_IT_UPDATE);
    LL_TIM_EnableIT_UPDATE(TIM12);

  #else //not USER_STM32
    TIMSK1 |= (1<<OCIE1A);
  #endif //USER_STM32
}


// Stepper shutdown
void st_go_idle()
{
  // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.
  #ifdef USER_STM32
    /* Disable the TIM Update interrupt */
    //__HAL_TIM_DISABLE_IT(&htim12, TIM_IT_UPDATE);
    LL_TIM_DisableIT_UPDATE(TIM12);

    /* Clear the IT pending Bit */
    //__HAL_TIM_CLEAR_IT(&htim12, TIM_IT_UPDATE);
    LL_TIM_ClearFlag_UPDATE(TIM12);

  #else //not USER_STM32
    TIMSK1 &= ~(1<<OCIE1A); // Disable Timer1 interrupt
    TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Reset clock to no prescaling.
  #endif //USER_STM32
  busy = false;

  // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
  bool pin_state = false; // Keep enabled.
  if (((settings.stepper_idle_lock_time != 0xff) || sys_rt_exec_alarm || sys.state == STATE_SLEEP) && sys.state != STATE_HOMING) {
    // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
    // stop and not drift from residual inertial forces at the end of the last movement.
    delay_ms(settings.stepper_idle_lock_time);
    pin_state = true; // Override. Disable steppers.
  }
  if (bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE)) { pin_state = !pin_state; } // Apply pin invert.
  #ifdef USER_STM32
    if (pin_state) {
      //__HAL_GPIO_SET_BITS(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);// Set pin to high
      LL_GPIO_SetOutputPin(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);
    }
    else {
      //__HAL_GPIO_RESET_BITS(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);// Set pin to low
      LL_GPIO_ResetOutputPin(STEPPERS_DISABLE_GPIO_Port, STEPPERS_DISABLE_Pin);
    }
  #else //not USER_STM32
    if (pin_state) { STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT); }
    else { STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT); }
  #endif //USER_STM32
}


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of Grbl. Grbl employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the Arduino's capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step
   pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is
   particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but
   is usually not a physical problem at higher frequencies, although audible.
     To improve Bresenham multi-axis performance, Grbl uses what we call an Adaptive Multi-Axis
   Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
   AMASS artificially increases the Bresenham resolution without effecting the algorithm's
   innate exactness. AMASS adapts its resolution levels automatically depending on the step
   frequency to be executed, meaning that for even lower step frequencies the step smoothing
   level increases. Algorithmically, AMASS is acheived by a simple bit-shifting of the Bresenham
   step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift
   the Bresenham step event count, effectively multiplying it by 2, while the axis step counts
   remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
   non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is
   stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
   Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any
   of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the
   stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing
   issues with the Bresenham algorithm and does not significantly alter Grbl's performance, but
   in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed.
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease
   CPU overhead with bitshift integer operations.
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.

   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick,
   which for Grbl must be less than 33.3usec (@30kHz ISR rate). Oscilloscope measured time in
   ISR is 5usec typical and 25usec maximum, well below requirement.
   NOTE: This ISR expects at least one step to be executed per segment.
*/
// TODO: Replace direct updating of the int32 position counters in the ISR somehow. Perhaps use smaller
// int8 variables and update position counters only when a segment completes. This can get complicated
// with probing and homing cycles that require true real-time positions.

#ifdef USER_STM32
void Step_Direction(void)
{
  uint16_t tempDir;
  #ifndef USER_ODR
    if(st.dir_bits & X_DIRECTION_BIT_SIMULATE_INDEX){
      __HAL_GPIO_SET_BITS(X_DIRECTION_GPIO_Port, X_DIRECTION_Pin);}
    else{
      __HAL_GPIO_RESET_BITS(X_DIRECTION_GPIO_Port, X_DIRECTION_Pin);
    }

    if(st.dir_bits & Y_DIRECTION_BIT_SIMULATE_INDEX){
      __HAL_GPIO_SET_BITS(Y_DIRECTION_GPIO_Port, Y_DIRECTION_Pin);}
    else{
      __HAL_GPIO_RESET_BITS(Y_DIRECTION_GPIO_Port, Y_DIRECTION_Pin);}

    if(st.dir_bits & Z_DIRECTION_BIT_SIMULATE_INDEX){
      __HAL_GPIO_SET_BITS(Z_DIRECTION_GPIO_Port, Z_DIRECTION_Pin);
    }
    else{
      __HAL_GPIO_RESET_BITS(Z_DIRECTION_GPIO_Port, Z_DIRECTION_Pin);}
    #ifdef USER_MORE_AXIS
      if(st.dir_bits & A_DIRECTION_BIT_SIMULATE_INDEX){
        __HAL_GPIO_SET_BITS(A_DIRECTION_GPIO_Port, A_DIRECTION_Pin);}
      else{
        __HAL_GPIO_RESET_BITS(A_DIRECTION_GPIO_Port, A_DIRECTION_Pin);}
      #ifdef USER_6_AXIS
        if(st.dir_bits & B_DIRECTION_BIT_SIMULATE_INDEX){
          __HAL_GPIO_SET_BITS(B_DIRECTION_GPIO_Port, B_DIRECTION_Pin);}
        else{
          __HAL_GPIO_RESET_BITS(B_DIRECTION_GPIO_Port, B_DIRECTION_Pin);}
        if(st.dir_bits & C_DIRECTION_BIT_SIMULATE_INDEX){
          __HAL_GPIO_SET_BITS(C_DIRECTION_GPIO_Port, C_DIRECTION_Pin);}
        else{
          __HAL_GPIO_RESET_BITS(C_DIRECTION_GPIO_Port, C_DIRECTION_Pin);}
        #ifdef USER_8_AXIS
          if(st.dir_bits & D_DIRECTION_BIT_SIMULATE_INDEX){
            __HAL_GPIO_SET_BITS(D_DIRECTION_GPIO_Port, D_DIRECTION_Pin);}
          else{
            __HAL_GPIO_RESET_BITS(D_DIRECTION_GPIO_Port, D_DIRECTION_Pin);}
          if(st.dir_bits & E_DIRECTION_BIT_SIMULATE_INDEX){
            __HAL_GPIO_SET_BITS(E_DIRECTION_GPIO_Port, E_DIRECTION_Pin);}
          else{
            __HAL_GPIO_RESET_BITS(E_DIRECTION_GPIO_Port, E_DIRECTION_Pin);}
        #endif
      #endif
    #endif
  #else
    #ifdef STM32H743xx
      #ifdef USER_PULSE_NOT_EQUAL_DIR
        tempDir = (DIRECTION_GPIO_Port->ODR) & 0xFFF0;
        DIRECTION_GPIO_Port->ODR = tempDir | ((uint16_t)st.dir_bits);
      #else
        tempDir = DIRECTION_GPIO_Port->ODR & DIRECTION_MASK_PORT;
        DIRECTION_GPIO_Port->ODR = tempDir | ((uint16_t)st.dir_bits << 8);
      #endif
    #else
      #ifdef USER_PULSE_NOT_EQUAL_DIR
        tempDir = DIRECTION_GPIO_Port->ODR & 0xFFF0;
        DIRECTION_GPIO_Port->ODR = tempDir | ((uint16_t)st.dir_bits);
      #else
        tempDir = DIRECTION_GPIO_Port->ODR & 0xFF0F;
        DIRECTION_GPIO_Port->ODR = tempDir | ((uint16_t)st.dir_bits << 4);
      #endif
    #endif
  #endif
}
  #ifndef USER_STEP_PWM
  void Step_Pulse(void)
  {
    uint16_t tempStep;
    #ifndef USER_ODR
      if(st.step_bits & X_STEP_BIT_SIMULATE_INDEX){
        __HAL_GPIO_SET_BITS(X_STEP_GPIO_Port, X_STEP_Pin);}
      else{
        __HAL_GPIO_RESET_BITS(X_STEP_GPIO_Port, X_STEP_Pin);}

      if(st.step_bits & Y_STEP_BIT_SIMULATE_INDEX){
        __HAL_GPIO_SET_BITS(Y_STEP_GPIO_Port, Y_STEP_Pin);}
      else{
        __HAL_GPIO_RESET_BITS(Y_STEP_GPIO_Port, Y_STEP_Pin);}

      if(st.step_bits & Z_STEP_BIT_SIMULATE_INDEX){
        __HAL_GPIO_SET_BITS(Z_STEP_GPIO_Port, Z_STEP_Pin);}
      else{
        __HAL_GPIO_RESET_BITS(Z_STEP_GPIO_Port, Z_STEP_Pin);}
      #ifdef USER_MORE_AXIS
        if(st.step_bits & A_STEP_BIT_SIMULATE_INDEX){
          __HAL_GPIO_SET_BITS(A_STEP_GPIO_Port, A_STEP_Pin);}
        else{
          __HAL_GPIO_SET_BITS(A_STEP_GPIO_Port, A_STEP_Pin);}
        #ifdef USER_6_AXIS
          if(st.step_bits & B_STEP_BIT_SIMULATE_INDEX){
            __HAL_GPIO_SET_BITS(B_STEP_GPIO_Port, B_STEP_Pin);}
          else{
            __HAL_GPIO_SET_BITS(B_STEP_GPIO_Port, B_STEP_Pin);}
          if(st.step_bits & C_STEP_BIT_SIMULATE_INDEX){
            __HAL_GPIO_SET_BITS(C_STEP_GPIO_Port, C_STEP_Pin);}
          else{
            __HAL_GPIO_SET_BITS(C_STEP_GPIO_Port, C_STEP_Pin);}
          #ifdef USER_8_AXIS
            if(st.step_bits & D_STEP_BIT_SIMULATE_INDEX){
              __HAL_GPIO_SET_BITS(D_STEP_GPIO_Port, D_STEP_Pin);}
            else{
              __HAL_GPIO_SET_BITS(D_STEP_GPIO_Port, D_STEP_Pin);}
            if(st.step_bits & E_STEP_BIT_SIMULATE_INDEX){
              __HAL_GPIO_SET_BITS(E_STEP_GPIO_Port, E_STEP_Pin);}
            else{
              __HAL_GPIO_SET_BITS(E_STEP_GPIO_Port, E_STEP_Pin);}
          #endif
        #endif
      #endif
    #else
      #ifdef STM32H743xx
        #ifdef USER_PULSE_NOT_EQUAL_DIR
          tempStep = STEP_GPIO_Port->ODR & 0xFFF0;
          STEP_GPIO_Port->ODR = tempStep | ((uint16_t)st.step_bits);
        #else
          tempStep = STEP_GPIO_Port->ODR & STEP_MASK_PORT;
          STEP_GPIO_Port->ODR = tempStep | ((uint16_t)st.step_bits);
        #endif
      #else
        #ifdef USER_PULSE_NOT_EQUAL_DIR
          tempStep = STEP_GPIO_Port->ODR & 0xFFF0;
          STEP_GPIO_Port->ODR = tempStep | ((uint16_t)st.step_bits);
        #else
          tempStep = STEP_GPIO_Port->ODR & 0xFFF0;
          STEP_GPIO_Port->ODR = tempStep | ((uint16_t)st.step_bits);
        #endif
      #endif
    #endif
  }
#else //USER_STEP_PWM
  void StepPulsePwmReset(void)
  {
    /* Sets the TIM Capture Compare Register */
    __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, st.step_pulse_time);
    __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, st.step_pulse_time);
    __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, st.step_pulse_time);
    #ifdef USER_MORE_AXIS
    __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, st.step_pulse_time);
    #ifdef USER_6_AXIS
      __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, st.step_pulse_time);
      __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, st.step_pulse_time);
      #ifdef USER_8_AXIS
        __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, st.step_pulse_time);
        __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, st.step_pulse_time);
      #endif
    #endif
    #endif//USER_MORE_AXIS
    /* Enable the Peripheral */
    __HAL_TIM_ENABLE(&htim1);
    #ifdef USER_6_AXIS
      /* Enable the Peripheral */
      __HAL_TIM_ENABLE(&htim8);
    #endif
  }
  inline void StepPulsePwm(void)
  {
  //  __HAL_TIM_SET_COUNTER(&htim1, 1);
    if(st.step_bits != 0)
    {
      /* Sets the TIM Capture Compare Register */
      if((st.step_bits & X_STEP_BIT_SIMULATE_INDEX)==X_STEP_BIT_SIMULATE_INDEX){
        __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, st.step_pulse_time_half);}
      else{
        __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, st.step_pulse_time);}

      if((st.step_bits & Y_STEP_BIT_SIMULATE_INDEX)==Y_STEP_BIT_SIMULATE_INDEX){
        __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, st.step_pulse_time_half);}
      else{
        __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, st.step_pulse_time);}

      if((st.step_bits & Z_STEP_BIT_SIMULATE_INDEX)==Z_STEP_BIT_SIMULATE_INDEX){
        __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, st.step_pulse_time_half);}
      else{
        __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, st.step_pulse_time);}

      #ifdef USER_MORE_AXIS
        if((st.step_bits & A_STEP_BIT_SIMULATE_INDEX)==A_STEP_BIT_SIMULATE_INDEX){
          __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, st.step_pulse_time_half);}
        else{
          __MY_HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, st.step_pulse_time);}
        #ifdef USER_6_AXIS
          if((st.step_bits & B_STEP_BIT_SIMULATE_INDEX)==B_STEP_BIT_SIMULATE_INDEX){
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, st.step_pulse_time_half);}
          else{
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, st.step_pulse_time);}
          if((st.step_bits & C_STEP_BIT_SIMULATE_INDEX)==C_STEP_BIT_SIMULATE_INDEX){
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, st.step_pulse_time_half);}
          else{
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, st.step_pulse_time);}
        #endif
        #ifdef USER_8_AXIS
          if((st.step_bits & D_STEP_BIT_SIMULATE_INDEX)==D_STEP_BIT_SIMULATE_INDEX){
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, st.step_pulse_time_half);}
          else{
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, st.step_pulse_time);}
          if((st.step_bits & E_STEP_BIT_SIMULATE_INDEX)==E_STEP_BIT_SIMULATE_INDEX){
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, st.step_pulse_time_half);}
          else{
            __MY_HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, st.step_pulse_time);}
        #endif
      #endif//USER_MORE_AXIS

      /* Enable the Peripheral */
      __HAL_TIM_ENABLE(&htim1);
      #ifdef USER_6_AXIS
        /* Enable the Peripheral */
        __HAL_TIM_ENABLE(&htim8);
      #endif
    }
    else{
      StepPulsePwmReset();
    }
  }
#endif //USER_STEP_PWM

void CalculateSysPosition(uint8_t idx)
{
  #ifdef USER_MORE_AXIS
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      st.counter[idx] += st.steps[idx];
    #else
      st.counter[idx] += st.exec_block->steps[idx];
    #endif
    if (st.counter[idx] > st.exec_block->step_event_count) {
      #ifdef USER_STM32
        bit_true(st.step_outbits, StepBitSimulateArray[idx]);
      #else //not USER_STM32
        st.step_outbits |= (1<<?_STEP_BIT); #error "Step no axis"
      #endif //USER_STM32
      st.counter[idx] -= st.exec_block->step_event_count;
      #ifdef USER_STM32
        if (st.exec_block->direction_bits & (DirectionBitSimulateArray[idx])) { sys_position[idx]--; }
      #else //not USER_STM32
        if (st.exec_block->direction_bits & (1<<?_DIRECTION_BIT)) { sys.position[?]--; }
      #endif //USER_STM32
      else { sys_position[idx]++; }
    }
  #else
  #endif
}

#endif
#ifdef USER_STM32
  void ISR_TIMER1_COMPA_vect(void)
#else //not USER_STM32
  ISR(TIMER1_COMPA_vect)
#endif //USER_STM32
{
  if (busy) { return; } // The busy-flag is used to avoid reentering this interrupt
  #ifdef USER_STM32
    // Set the direction pins a couple of nanoseconds before we step the steppers
    #ifdef USER_NOT_STEP_PORT_INV
      #ifdef USER_8_AXIS
        st.dir_bits = st.dir_outbits;
      #else
        st.dir_bits = (st.dir_outbits & DIRECTION_MASK_SIMULATE);
      #endif
    #else
      st.dir_bits = (st.dir_bits & DIRECTION_MASK_SIMULATE_INV) | (st.dir_outbits & DIRECTION_MASK_SIMULATE);
    #endif
    Step_Direction();

    #ifdef STEP_PULSE_DELAY
      // Store out_bits to prevent overwriting.
      #ifdef USER_NOT_STEP_PORT_INV
        #ifdef USER_8_AXIS
          st.step_bits = st.step_outbits;
        #else
          st.step_bits = st.step_outbits & STEP_MASK_SIMULATE;
        #endif
      #else
        st.step_bits = (st.step_bits & STEP_MASK_SIMULATE_INV) | st.step_outbits;
      #endif
      #ifdef USER_STEP_PWM
      #else
        #ifdef USER_STEP_PULSE_DUTY_CHANGE
          st.step_pulse_time = st.exec_segment->cycles_per_tick << 1;
          if(st.step_pulse_time < TICKS_PER_MICROSECOND_DELAY) //2.0 uS
          st.step_pulse_time = TICKS_PER_MICROSECOND_DELAY;
          __MY_HAL_TIM_SET_AUTORELOAD(&htim14, st.step_pulse_time - 1);
          __HAL_TIM_SET_COUNTER(&htim14, (st.step_pulse_time/2)-1);
        #else
          //__HAL_TIM_SET_COUNTER(&htim14, st.step_pulse_time_half);
          LL_TIM_SetCounter(TIM14, st.step_pulse_time_half);
        #endif //USER_STEP_PULSE_DUTY_CHANGE
      #endif

    #else //not STEP_PULSE_DELAY
      __HAL_TIM_SET_COUNTER(&htim14, (st.step_pulse_time_half*2)); // Reload Timer0 counter
    #endif //STEP_PULSE_DELAY

    #ifdef USER_STEP_PWM
      //      st.step_bits = step_port_invert_mask;
    #else
      /* Clear the IT pending Bit */
      //__HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
      LL_TIM_ClearFlag_UPDATE(TIM14);

      /* Enable the TIM Update interrupt */
      //__HAL_TIM_ENABLE_IT(&htim14, TIM_IT_UPDATE);
      LL_TIM_EnableIT_UPDATE(TIM14);
    #endif

    // Then pulse the stepping pins
    #ifdef STEP_PULSE_DELAY
      #ifdef USER_STEP_PWM
        StepPulsePwm();
      #endif
    #else  // Normal operation
      st.step_bits = (st.step_bits & ~STEP_MASK_SIMULATE) | st.step_outbits;
      Step_Pulse();
    #endif
  #else //not USER_STM32
    // Set the direction pins a couple of nanoseconds before we step the steppers
    DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | (st.dir_outbits & DIRECTION_MASK);

    // Then pulse the stepping pins
    #ifdef STEP_PULSE_DELAY
      st.step_bits = (STEP_PORT & ~STEP_MASK) | st.step_outbits; // Store out_bits to prevent overwriting.
    #else  // Normal operation
      STEP_PORT = (STEP_PORT & ~STEP_MASK) | st.step_outbits;
    #endif

    // Enable step pulse reset timer so that The Stepper Port Reset Interrupt can reset the signal after
    // exactly settings.pulse_microseconds microseconds, independent of the main Timer1 prescaler.
    TCNT0 = st.step_pulse_time; // Reload Timer0 counter
    TCCR0B = (1<<CS01); // Begin Timer0. Full speed, 1/8 prescaler
  #endif //USER_STM32
  busy = true;
  sei(); // Re-enable interrupts to allow Stepper Port Reset Interrupt to fire on-time.
         // NOTE: The remaining code in this ISR will finish before returning to main program.

  // If there is no step segment, attempt to pop one from the stepper buffer
  if (st.exec_segment == NULL) {
    // Anything in the buffer? If so, load and initialize next step segment.
    if (segment_buffer_head != segment_buffer_tail) {
      // Initialize new step segment and load number of steps to execute
      st.exec_segment = &segment_buffer[segment_buffer_tail];

      #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        #ifdef USER_STM32
          //CS10 atemega 8 bit 0 on TCCR1B---> in grbl use 1,8,64
          //prescaler=1,8,64 => 1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192;
          /* Set the Prescaler value */
          //TIM1->PSC = (((2^x) *5) -1);
          uint16_t temp_prescaler = st.exec_segment->prescaler * MUL_TIMER10 - 1;
          //__HAL_TIM_SET_PRESCALER(&htim12, temp_prescaler);
          LL_TIM_SetPrescaler(TIM12, temp_prescaler);
        #else //not USER_STM32
          // With AMASS is disabled, set timer prescaler for segments with slow step frequencies (< 250Hz).
          TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (st.exec_segment->prescaler<<CS10);
        #endif //USER_STM32
      #endif

      // Initialize step segment timing per step and load number of steps to execute.
      #ifdef USER_STM32
        //__MY_HAL_TIM_SET_AUTORELOAD(&htim12, st.exec_segment->cycles_per_tick);
        LL_TIM_SetAutoReload(TIM12, st.exec_segment->cycles_per_tick);

      #else //not USER_STM32
        OCR1A = st.exec_segment->cycles_per_tick;
      #endif //USER_STM32
      st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.
      // If the new segment starts a new planner block, initialize stepper variables and counters.
      // NOTE: When the segment data index changes, this indicates a new planner block.
      if ( st.exec_block_index != st.exec_segment->st_block_index ) {
        st.exec_block_index = st.exec_segment->st_block_index;
        st.exec_block = &st_block_buffer[st.exec_block_index];

        // Initialize Bresenham line and distance counters
        #ifdef USER_MORE_AXIS
            st.counter[X_AXIS] = (st.exec_block->step_event_count >> 1);
            st.counter[Y_AXIS] = st.counter[X_AXIS];
            st.counter[Z_AXIS] = st.counter[X_AXIS];
            st.counter[A_AXIS] = st.counter[X_AXIS];
            #ifdef USER_6_AXIS
              st.counter[B_AXIS] = st.counter[X_AXIS];
              st.counter[C_AXIS] = st.counter[X_AXIS];
              #ifdef USER_8_AXIS
                st.counter[D_AXIS] = st.counter[X_AXIS];
                st.counter[E_AXIS] = st.counter[X_AXIS];
              #endif
            #endif
        #else //not USER_MORE_AXIS
          st.counter_x = st.counter_y = st.counter_z = (st.exec_block->step_event_count >> 1);
        #endif //USER_MORE_AXIS
      }
      st.dir_outbits = st.exec_block->direction_bits ^ dir_port_invert_mask;

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
        st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.exec_segment->amass_level;
        st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.exec_segment->amass_level;
        st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.exec_segment->amass_level;
		#ifdef USER_MORE_AXIS
            st.steps[A_AXIS] = st.exec_block->steps[A_AXIS] >> st.exec_segment->amass_level;
            #ifdef USER_6_AXIS
              st.steps[B_AXIS] = st.exec_block->steps[B_AXIS] >> st.exec_segment->amass_level;
              st.steps[C_AXIS] = st.exec_block->steps[C_AXIS] >> st.exec_segment->amass_level;
              #ifdef USER_8_AXIS
                st.steps[D_AXIS] = st.exec_block->steps[D_AXIS] >> st.exec_segment->amass_level;
                st.steps[E_AXIS] = st.exec_block->steps[E_AXIS] >> st.exec_segment->amass_level;
              #endif
            #endif
        #endif //USER_MORE_AXIS
      #endif

      #ifdef VARIABLE_SPINDLE
        // Set real-time spindle output as segment is loaded, just prior to the first step.
        spindle_set_speed(st.exec_segment->spindle_pwm);
      #endif

    } else {
      // Segment buffer empty. Shutdown.
      st_go_idle();
      #ifdef VARIABLE_SPINDLE
        // Ensure pwm is set properly upon completion of rate-controlled motion.
        if (st.exec_block->is_pwm_rate_adjusted) { spindle_set_speed(SPINDLE_PWM_OFF_VALUE); }
      #endif
      system_set_exec_state_flag(EXEC_CYCLE_STOP); // Flag main program for cycle end
      return; // Nothing to do but exit.
    }
  }
  #ifdef CHECK_PROBE_IN_INTERRUPT
    // Check probing state.
    if (sys_probe_state == PROBE_ACTIVE) { probe_state_monitor(); }
  #endif
  // Reset step out bits.
  st.step_outbits = 0;

  // Execute step displacement profile by Bresenham line algorithm
  #ifdef USER_MORE_AXIS
    CalculateSysPosition(X_AXIS);
    CalculateSysPosition(Y_AXIS);
    CalculateSysPosition(Z_AXIS);
    CalculateSysPosition(A_AXIS);
    #ifdef USER_6_AXIS
      CalculateSysPosition(B_AXIS);
      CalculateSysPosition(C_AXIS);
      #ifdef USER_8_AXIS
        CalculateSysPosition(D_AXIS);
        CalculateSysPosition(E_AXIS);
      #endif
    #endif
  #else //not USER_MORE_AXIS
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      st.counter_x += st.steps[X_AXIS];
    #else
      st.counter_x += st.exec_block->steps[X_AXIS];
    #endif
    if (st.counter_x > st.exec_block->step_event_count) {
      #ifdef USER_STM32
        bit_true(st.step_outbits, X_STEP_BIT_SIMULATE_INDEX);
      #else //not USER_STM32
        st.step_outbits |= (1<<X_STEP_BIT);
      #endif //USER_STM32
        st.counter_x -= st.exec_block->step_event_count;
      #ifdef USER_STM32
        if (st.exec_block->direction_bits & (X_DIRECTION_BIT_SIMULATE_INDEX)) { sys_position[X_AXIS]--; }
      #else //not USER_STM32
        if (st.exec_block->direction_bits & (1<<X_DIRECTION_BIT)) { sys_position[X_AXIS]--; }
      #endif //USER_STM32
      else { sys_position[X_AXIS]++; }
    }
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      st.counter_y += st.steps[Y_AXIS];
    #else
      st.counter_y += st.exec_block->steps[Y_AXIS];
    #endif
    if (st.counter_y > st.exec_block->step_event_count) {
      #ifdef USER_STM32
        bit_true(st.step_outbits, Y_STEP_BIT_SIMULATE_INDEX);
      #else //not USER_STM32
        st.step_outbits |= (1<<Y_STEP_BIT);
      #endif //USER_STM32
      st.counter_y -= st.exec_block->step_event_count;
      #ifdef USER_STM32
        if (st.exec_block->direction_bits & (Y_DIRECTION_BIT_SIMULATE_INDEX)) { sys_position[Y_AXIS]--; }
      #else //not USER_STM32
        if (st.exec_block->direction_bits & (1<<Y_DIRECTION_BIT)) { sys_position[Y_AXIS]--; }
      #endif //USER_STM32
        else { sys_position[Y_AXIS]++; }
    }
    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      st.counter_z += st.steps[Z_AXIS];
    #else
      st.counter_z += st.exec_block->steps[Z_AXIS];
    #endif
    if (st.counter_z > st.exec_block->step_event_count) {
      #ifdef USER_STM32
        bit_true(st.step_outbits, Z_STEP_BIT_SIMULATE_INDEX);
      #else //not USER_STM32
        st.step_outbits |= (1<<Z_STEP_BIT);
      #endif //USER_STM32
      st.counter_z -= st.exec_block->step_event_count;
      #ifdef USER_STM32
        if (st.exec_block->direction_bits & (Z_DIRECTION_BIT_SIMULATE_INDEX)) { sys_position[Z_AXIS]--; }
      #else //not USER_STM32
        if (st.exec_block->direction_bits & (1<<Z_DIRECTION_BIT)) { sys_position[Z_AXIS]--; }
      #endif //USER_STM32
        else { sys_position[Z_AXIS]++; }
    }
  #endif //USER_MORE_AXIS
  #ifdef USER_SAVE_POSITION
    BufferBackupRTC32[X_AXIS] = sys_position[X_AXIS];
    BufferBackupRTC32[Y_AXIS] = sys_position[Y_AXIS];
    BufferBackupRTC32[Z_AXIS] = sys_position[Z_AXIS];
    #ifdef USER_MORE_AXIS
      BufferBackupRTC32[A_AXIS] = sys_position[A_AXIS];
      #ifdef USER_6_AXIS
        BufferBackupRTC32[B_AXIS] = sys_position[B_AXIS];
        BufferBackupRTC32[C_AXIS] = sys_position[C_AXIS];
        #ifdef USER_8_AXIS
          BufferBackupRTC32[D_AXIS] = sys_position[D_AXIS];
          BufferBackupRTC32[E_AXIS] = sys_position[E_AXIS];
        #endif
      #endif
    #endif //USER_MORE_AXIS
  #endif //USER_SAVE_POSITION
  // During a homing cycle, lock out and prevent desired axes from moving.
  if (sys.state == STATE_HOMING) { st.step_outbits &= sys.homing_axis_lock; }

  st.step_count--; // Decrement step events count
  if (st.step_count == 0) {
    // Segment is complete. Discard current segment and advance segment indexing.
    st.exec_segment = NULL;
    if ( ++segment_buffer_tail == SEGMENT_BUFFER_SIZE) { segment_buffer_tail = 0; }
  }
  #ifdef USER_NOT_STEP_PORT_INV
  #else
    st.step_outbits ^= step_port_invert_mask;  // Apply step port invert mask
  #endif
  busy = false;
}


/* The Stepper Port Reset Interrupt: Timer0 OVF interrupt handles the falling edge of the step
   pulse. This should always trigger before the next Timer1 COMPA interrupt and independently
   finish, if Timer1 is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/
// This interrupt is enabled by ISR_TIMER1_COMPAREA when it sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
#ifdef USER_STM32
  #ifdef  USER_STEP_PWM
  #else
    void ISR_TIMER0_OVF_vect(void)
    {
      // Reset stepping pins (leave the direction pins)
      #ifdef USER_NOT_STEP_PORT_INV
        st.step_bits = step_port_invert_mask;//(step_port_invert_mask & STEP_MASK_SIMULATE);
      #else
        st.step_bits = (st.step_bits & STEP_MASK_SIMULATE_INV) | (step_port_invert_mask & STEP_MASK_SIMULATE);
      #endif
      Step_Pulse();
      /* Disable the TIM Update interrupt */
      //__HAL_TIM_DISABLE_IT(&htim14, TIM_IT_UPDATE);
      LL_TIM_DisableIT_UPDATE(TIM14);

    }
  #endif
#else //not USER_STM32
  ISR(TIMER0_OVF_vect)
  {
    // Reset stepping pins (leave the direction pins)
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | (step_port_invert_mask & STEP_MASK);
    TCCR0B = 0; // Disable Timer0 to prevent re-entering this interrupt when it's not needed.
  }
#endif //USER_STM32
#ifdef STEP_PULSE_DELAY
  // This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
  // initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
  // will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
  // The new timing between direction, step pulse, and step complete events are setup in the
  // st_wake_up() routine.
  #ifdef USER_STM32
    #ifdef USER_STEP_PWM
    #else
      void ISR_TIMER0_COMPA_vect(void)
      {
        Step_Pulse();
      }
    #endif
  #else //not USER_STM32
    ISR(TIMER0_COMPA_vect)
    {
      STEP_PORT = st.step_bits; // Begin step pulse.
    }
  #endif //USER_STM32
#endif


// Generates the step and direction port invert masks used in the Stepper Interrupt Driver.
void st_generate_step_dir_invert_masks()
{
  uint8_t idx;
  step_port_invert_mask = 0;
  dir_port_invert_mask = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef USER_NOT_STEP_PORT_INV
    //step_port_invert_mask = 0;
    //first had
    #else
      if (bit_istrue(settings.step_invert_mask,bit(idx))) { step_port_invert_mask |= get_step_pin_mask(idx); }
    #endif
    if (bit_istrue(settings.dir_invert_mask,bit(idx))) { dir_port_invert_mask |= get_direction_pin_mask(idx); }
  }
}


// Reset and clear stepper subsystem variables
void st_reset()
{
  // Initialize stepper driver idle state.
  st_go_idle();

  // Initialize stepper algorithm variables.
  memset(&prep, 0, sizeof(st_prep_t));
  memset(&st, 0, sizeof(stepper_t));
  st.exec_segment = NULL;
  pl_block = NULL;  // Planner block pointer used by segment buffer
  segment_buffer_tail = 0;
  segment_buffer_head = 0; // empty = tail
  segment_next_head = 1;
  busy = false;

  st_generate_step_dir_invert_masks();
  st.dir_outbits = dir_port_invert_mask; // Initialize direction bits to default.

  // Initialize step and direction port pins.
  #ifdef USER_STM32
    // Initialize step and direction port pins.
    #ifdef USER_NOT_STEP_PORT_INV
      st.step_bits = step_port_invert_mask;
      #ifdef USER_STEP_PWM
        ComputationStepDelay();
        StepPulsePwmReset();
      #endif
    #else
      st.step_bits = (st.step_bits & STEP_MASK_SIMULATE_INV) | step_port_invert_mask;
    #endif
    #ifdef USER_STEP_PWM
    #else
      Step_Pulse();
    #endif
    #ifdef USER_NOT_STEP_PORT_INV
      st.dir_bits = dir_port_invert_mask;
    #else
      st.dir_bits = (st.dir_bits & DIRECTION_MASK_SIMULATE_INV) | dir_port_invert_mask;
    #endif
    Step_Direction();
  #else //not USER_STM32
    STEP_PORT = (STEP_PORT & ~STEP_MASK) | step_port_invert_mask;
    DIRECTION_PORT = (DIRECTION_PORT & ~DIRECTION_MASK) | dir_port_invert_mask;
  #endif //USER_STM32
}


// Initialize and start the stepper motor subsystem
#ifdef USER_STM32
  void stepper_init()
  {
    #ifdef USER_OUTPUT_OPEN_DRAIN
      GPIO_InitTypeDef GPIO_InitStruct;
      /*Configure GPIO pins : X_STEP_Pin Y_STEP_Pin Z_STEP_Pin A_STEP_Pin */
      #ifdef USER_MORE_AXIS
        #ifdef USER_6_AXIS
          #ifdef USER_8_AXIS
            GPIO_InitStruct.Pin = X_STEP_Pin|Y_STEP_Pin|Z_STEP_Pin|A_STEP_Pin|B_STEP_Pin|C_STEP_Pin|D_STEP_Pin|E_STEP_Pin;
          #else
            GPIO_InitStruct.Pin = X_STEP_Pin|Y_STEP_Pin|Z_STEP_Pin|A_STEP_Pin|B_STEP_Pin|C_STEP_Pin;
          #endif
        #else
          GPIO_InitStruct.Pin = X_STEP_Pin|Y_STEP_Pin|Z_STEP_Pin|A_STEP_Pin;
        #endif
      #else //not USER_MORE_AXIS
        GPIO_InitStruct.Pin = X_STEP_Pin|Y_STEP_Pin|Z_STEP_Pin;
        HAL_GPIO_DeInit(STEP_GPIO_Port, A_STEP_Pin);
        #ifdef USER_6_AXIS
          HAL_GPIO_DeInit(STEP_GPIO_Port, B_STEP_Pin);
          HAL_GPIO_DeInit(STEP_GPIO_Port, C_STEP_Pin);
        #endif
      #endif //USER_MORE_AXIS
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      HAL_GPIO_Init(STEP_GPIO_Port, &GPIO_InitStruct);

      /*Configure GPIO pins : X_DIRECTION_Pin Y_DIRECTION_Pin Z_DIRECTION_Pin A_DIRECTION_Pin */
      #ifdef USER_MORE_AXIS
        #ifdef USER_6_AXIS
          #ifdef USER_8_AXIS
            GPIO_InitStruct.Pin = X_DIRECTION_Pin|Y_DIRECTION_Pin|Z_DIRECTION_Pin|A_DIRECTION_Pin|B_DIRECTION_Pin|C_DIRECTION_Pin|D_DIRECTION_Pin|E_DIRECTION_Pin;
          #else
            GPIO_InitStruct.Pin = X_DIRECTION_Pin|Y_DIRECTION_Pin|Z_DIRECTION_Pin|A_DIRECTION_Pin|B_DIRECTION_Pin|C_DIRECTION_Pin;
          #endif
        #else
          GPIO_InitStruct.Pin = X_DIRECTION_Pin|Y_DIRECTION_Pin|Z_DIRECTION_Pin|A_DIRECTION_Pin;
        #endif
      #else //not USER_MORE_AXIS
        GPIO_InitStruct.Pin = X_DIRECTION_Pin|Y_DIRECTION_Pin|Z_DIRECTION_Pin;
        HAL_GPIO_DeInit(DIRECTION_GPIO_Port, A_DIRECTION_Pin);
      #endif //USER_MORE_AXIS
      HAL_GPIO_Init(DIRECTION_GPIO_Port, &GPIO_InitStruct);

      /*Configure GPIO pins : STEPPERS_DISABLE_Pin */
      GPIO_InitStruct.Pin = STEPPERS_DISABLE_Pin;
      HAL_GPIO_Init(STEPPERS_DISABLE_GPIO_Port, &GPIO_InitStruct);
    #else
  //    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    #endif

    #ifdef USER_STEP_PWM
      //  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
      //  HAL_TIMEx_OCN_Start(&htim1, TIM_CHANNEL_1);
      //  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2)

//        /* Enable the Capture compare channel N */
//        TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
//        /* Enable the Capture compare channel N */
//        TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
//        /* Enable the Capture compare channel N */
//        TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
        /* Enable the Capture compare channel */
        TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
        /* Enable the Capture compare channel */
        TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
        /* Enable the Capture compare channel */
        TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
        #ifdef USER_MORE_AXIS
          /* Enable the Capture compare channel */
          TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
          #ifdef USER_6_AXIS
            /* Enable the Capture compare channel */
            TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
            /* Enable the Capture compare channel */
            TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
            #ifdef USER_8_AXIS
              /* Enable the Capture compare channel */
              TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);
              /* Enable the Capture compare channel */
              TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
            #endif
          #endif
        #endif

        /* Enable the main output */
        __HAL_TIM_MOE_ENABLE(&htim1);
        /* Clear the IT pending Bit */
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
        /* Disable the TIM Update interrupt */
        __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
        #ifdef USER_6_AXIS
        /* Enable the main output */
        __HAL_TIM_MOE_ENABLE(&htim8);
        /* Clear the IT pending Bit */
        __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
        /* Disable the TIM Update interrupt */
        __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
        #endif
    #else
      /* Disable the TIM Update interrupt */
      //__HAL_TIM_DISABLE_IT(&htim14, TIM_IT_UPDATE);
      LL_TIM_DisableIT_UPDATE(TIM14);

      /* Clear the IT pending Bit */
      //__HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
      LL_TIM_ClearFlag_UPDATE(TIM14);

      /* Enable the Peripheral */
      //__HAL_TIM_ENABLE(&htim14);
      LL_TIM_EnableCounter(TIM14);
    #endif

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      uint16_t temp =  1 * MUL_TIMER10 - 1;//st.exec_segment->prescaler
      __MY_HAL_TIM_SET_AUTORELOAD(&htim12, AMASS_LEVEL0);//st.exec_segment->cycles_per_tick// Just set the highest speed possible.
      LL_TIM_SetAutoReload(TIM12, AMASS_LEVEL0);
    #else
      uint16_t temp =  ((1 << 0) * MUL_TIMER10) - 1;//st.exec_segment->prescaler
      //__MY_HAL_TIM_SET_AUTORELOAD(&htim12, 250);//st.exec_segment->cycles_per_tick// Just set the highest speed possible.
      LL_TIM_SetAutoReload(TIM12, 250);
    #endif

    //__HAL_TIM_SET_PRESCALER(&htim12, temp);
    LL_TIM_SetPrescaler(TIM12, temp);

    /* Enable the Peripheral */
    //__HAL_TIM_ENABLE(&htim12);
    LL_TIM_EnableCounter(TIM12);
  }
#else //not USER_STM32
  void stepper_init()
  {
    // Configure step and direction interface pins
    STEP_DDR |= STEP_MASK;
    STEPPERS_DISABLE_DDR |= 1<<STEPPERS_DISABLE_BIT;
    DIRECTION_DDR |= DIRECTION_MASK;

    // Configure Timer 1: Stepper Driver Interrupt
    TCCR1B &= ~(1<<WGM13); // waveform generation = 0100 = CTC
    TCCR1B |=  (1<<WGM12);
    TCCR1A &= ~((1<<WGM11) | (1<<WGM10));
    TCCR1A &= ~((1<<COM1A1) | (1<<COM1A0) | (1<<COM1B1) | (1<<COM1B0)); // Disconnect OC1 output
    // TCCR1B = (TCCR1B & ~((1<<CS12) | (1<<CS11))) | (1<<CS10); // Set in st_go_idle().
    // TIMSK1 &= ~(1<<OCIE1A);  // Set in st_go_idle().

    // Configure Timer 0: Stepper Port Reset Interrupt
    TIMSK0 &= ~((1<<OCIE0B) | (1<<OCIE0A) | (1<<TOIE0)); // Disconnect OC0 outputs and OVF interrupt.
    TCCR0A = 0; // Normal operation
    TCCR0B = 0; // Disable Timer0 until needed
    TIMSK0 |= (1<<TOIE0); // Enable Timer0 overflow interrupt
    #ifdef STEP_PULSE_DELAY
      TIMSK0 |= (1<<OCIE0A); // Enable Timer0 Compare Match A interrupt
    #endif
  }
#endif //USER_STM32

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters()
{
  if (pl_block != NULL) { // Ignore if at start of a new block.
    prep.recalculate_flag |= PREP_FLAG_RECALCULATE;
    pl_block->entry_speed_sqr = prep.current_speed*prep.current_speed; // Update entry speed.
    pl_block = NULL; // Flag st_prep_segment() to load and check active velocity profile.
  }
}


// Increments the step segment buffer block data ring buffer.
static uint8_t st_next_block_index(uint8_t block_index)
{
  block_index++;
  if ( block_index == (SEGMENT_BUFFER_SIZE-1) ) { return(0); }
  return(block_index);
}


#ifdef PARKING_ENABLE
  // Changes the run state of the step segment buffer to execute the special parking motion.
  void st_parking_setup_buffer()
  {
    // Store step execution data of partially completed block, if necessary.
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      prep.last_st_block_index = prep.st_block_index;
      prep.last_steps_remaining = prep.steps_remaining;
      prep.last_dt_remainder = prep.dt_remainder;
      prep.last_step_per_mm = prep.step_per_mm;
    }
    // Set flags to execute a parking motion
    prep.recalculate_flag |= PREP_FLAG_PARKING;
    prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE);
    pl_block = NULL; // Always reset parking motion to reload new block.
  }


  // Restores the step segment buffer to the normal run state after a parking motion.
  void st_parking_restore_buffer()
  {
    // Restore step execution data and flags of partially completed block, if necessary.
    if (prep.recalculate_flag & PREP_FLAG_HOLD_PARTIAL_BLOCK) {
      st_prep_block = &st_block_buffer[prep.last_st_block_index];
      prep.st_block_index = prep.last_st_block_index;
      prep.steps_remaining = prep.last_steps_remaining;
      prep.dt_remainder = prep.last_dt_remainder;
      prep.step_per_mm = prep.last_step_per_mm;
      prep.recalculate_flag = (PREP_FLAG_HOLD_PARTIAL_BLOCK | PREP_FLAG_RECALCULATE);
      prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm; // Recompute this value.
    } else {
      prep.recalculate_flag = false;
    }
    pl_block = NULL; // Set to reload next block.
  }
#endif


/* Prepares step segment buffer. Continuously called from main program.

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it.
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
void st_prep_buffer()
{
  // Block step prep buffer, while in a suspend state and there is no suspend motion to execute.
  if (bit_istrue(sys.step_control,STEP_CONTROL_END_MOTION)) { return; }

  while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

    // Determine if we need to load a new planner block or if the block needs to be recomputed.
    if (pl_block == NULL) {

      // Query planner for a queued block
      if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) { pl_block = plan_get_system_motion_block(); }
      else { pl_block = plan_get_current_block(); }
      if (pl_block == NULL) { return; } // No planner blocks. Exit.

      // Check if we need to only recompute the velocity profile or load a new block.
      if (prep.recalculate_flag & PREP_FLAG_RECALCULATE) {

        #ifdef PARKING_ENABLE
          if (prep.recalculate_flag & PREP_FLAG_PARKING) { prep.recalculate_flag &= ~(PREP_FLAG_RECALCULATE); }
          else { prep.recalculate_flag = false; }
        #else
          prep.recalculate_flag = false;
        #endif

      } else {

        // Load the Bresenham stepping data for the block.
        prep.st_block_index = st_next_block_index(prep.st_block_index);

        // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
        // when the segment buffer completes the planner block, it may be discarded when the
        // segment buffer finishes the prepped block, but the stepper ISR is still executing it.
        st_prep_block = &st_block_buffer[prep.st_block_index];
        st_prep_block->direction_bits = pl_block->direction_bits;
        uint8_t idx;
        #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = (pl_block->steps[idx] << 1); }
          st_prep_block->step_event_count = (pl_block->step_event_count << 1);
        #else
          // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS
          // level, such that we never divide beyond the original data anywhere in the algorithm.
          // If the original data is divided, we can lose a step from integer roundoff.
          for (idx=0; idx<N_AXIS; idx++) { st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL; }
          st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
        #endif
        // Initialize segment buffer data for generating the segments.
        #ifdef USER_DOUBLE
          prep.steps_remaining = (double)pl_block->step_event_count;
        #else
          prep.steps_remaining = (float)pl_block->step_event_count;
        #endif
        prep.step_per_mm = prep.steps_remaining/pl_block->millimeters;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR/prep.step_per_mm;
        prep.dt_remainder = 0.0; // Reset for new segment block

        if ((sys.step_control & STEP_CONTROL_EXECUTE_HOLD) || (prep.recalculate_flag & PREP_FLAG_DECEL_OVERRIDE)) {
          // New block loaded mid-hold. Override planner block entry speed to enforce deceleration.
          prep.current_speed = prep.exit_speed;
          pl_block->entry_speed_sqr = prep.exit_speed*prep.exit_speed;
          prep.recalculate_flag &= ~(PREP_FLAG_DECEL_OVERRIDE);
        } else {
          prep.current_speed = sqrt(pl_block->entry_speed_sqr);
        }
        
        #ifdef VARIABLE_SPINDLE
          // Setup laser mode variables. PWM rate adjusted motions will always complete a motion with the
          // spindle off. 
          st_prep_block->is_pwm_rate_adjusted = false;
          if (settings.flags & BITFLAG_LASER_MODE) {
            if (pl_block->condition & PL_COND_FLAG_SPINDLE_CCW) { 
              // Pre-compute inverse programmed rate to speed up PWM updating per step segment.
              prep.inv_rate = 1.0/pl_block->programmed_rate;
              st_prep_block->is_pwm_rate_adjusted = true; 
            }
          }
        #endif
      }

      /* ---------------------------------------------------------------------------------
       Compute the velocity profile of a new planner block based on its entry and exit
       speeds, or recompute the profile of a partially-completed planner block if the
       planner has updated it. For a commanded forced-deceleration, such as from a feed
       hold, override the planner velocities and decelerate to the target exit speed.
      */
      prep.mm_complete = 0.0; // Default velocity profile complete at 0.0mm from end of block.
      #ifdef USER_DOUBLE
        double inv_2_accel = 0.5/pl_block->acceleration;
      #else
        float inv_2_accel = 0.5/pl_block->acceleration;
      #endif
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) { // [Forced Deceleration to Zero Velocity]
        // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
        // the planner block profile, enforcing a deceleration to zero speed.
        prep.ramp_type = RAMP_DECEL;
        // Compute decelerate distance relative to end of block.
        #ifdef USER_DOUBLE
          double decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        #else
          float decel_dist = pl_block->millimeters - inv_2_accel*pl_block->entry_speed_sqr;
        #endif
        if (decel_dist < 0.0) {
          // Deceleration through entire planner block. End of feed hold is not in this block.
          prep.exit_speed = sqrt(pl_block->entry_speed_sqr-2*pl_block->acceleration*pl_block->millimeters);
        } else {
          prep.mm_complete = decel_dist; // End of feed hold.
          prep.exit_speed = 0.0;
        }
      } else { // [Normal Operation]
        // Compute or recompute velocity profile parameters of the prepped planner block.
        prep.ramp_type = RAMP_ACCEL; // Initialize as acceleration ramp.
        prep.accelerate_until = pl_block->millimeters;
        #ifdef USER_DOUBLE
          double exit_speed_sqr;
          double nominal_speed;
        #else
          float exit_speed_sqr;
          float nominal_speed;
        #endif
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          prep.exit_speed = exit_speed_sqr = 0.0; // Enforce stop at end of system motion.
        } else {
          exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
          prep.exit_speed = sqrt(exit_speed_sqr);
        }

        nominal_speed = plan_compute_profile_nominal_speed(pl_block);

        #ifdef USER_DOUBLE
          double nominal_speed_sqr = nominal_speed*nominal_speed;
          double intersect_distance =
        #else
          float nominal_speed_sqr = nominal_speed*nominal_speed;
          float intersect_distance =
        #endif

        0.5*(pl_block->millimeters+inv_2_accel*(pl_block->entry_speed_sqr-exit_speed_sqr));

        if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.
          prep.accelerate_until = pl_block->millimeters - inv_2_accel*(pl_block->entry_speed_sqr-nominal_speed_sqr);
          if (prep.accelerate_until <= 0.0) { // Deceleration-only.
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;

            // Compute override block exit speed since it doesn't match the planner exit speed.
            prep.exit_speed = sqrt(pl_block->entry_speed_sqr - 2*pl_block->acceleration*pl_block->millimeters);
            prep.recalculate_flag |= PREP_FLAG_DECEL_OVERRIDE; // Flag to load next block as deceleration override.

            // TODO: Determine correct handling of parameters in deceleration-only.
            // Can be tricky since entry speed will be current speed, as in feed holds.
            // Also, look into near-zero speed handling issues with this.

          } else {
            // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr); // Should always be >= 0.0 due to planner reinit.
            prep.maximum_speed = nominal_speed;
            prep.ramp_type = RAMP_DECEL_OVERRIDE;
          }
        } else if (intersect_distance > 0.0) {
          if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
            // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
            prep.decelerate_after = inv_2_accel*(nominal_speed_sqr-exit_speed_sqr);
            if (prep.decelerate_after < intersect_distance) { // Trapezoid type
              prep.maximum_speed = nominal_speed;
              if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                // Cruise-deceleration or cruise-only type.
                prep.ramp_type = RAMP_CRUISE;
              } else {
                // Full-trapezoid or acceleration-cruise types
                prep.accelerate_until -= inv_2_accel*(nominal_speed_sqr-pl_block->entry_speed_sqr);
              }
            } else { // Triangle type
              prep.accelerate_until = intersect_distance;
              prep.decelerate_after = intersect_distance;
              prep.maximum_speed = sqrt(2.0*pl_block->acceleration*intersect_distance+exit_speed_sqr);
            }
          } else { // Deceleration-only type
            prep.ramp_type = RAMP_DECEL;
            // prep.decelerate_after = pl_block->millimeters;
            // prep.maximum_speed = prep.current_speed;
          }
        } else { // Acceleration-only type
          prep.accelerate_until = 0.0;
          // prep.decelerate_after = 0.0;
          prep.maximum_speed = prep.exit_speed;
        }
      }
      
      #ifdef VARIABLE_SPINDLE
        bit_true(sys.step_control, STEP_CONTROL_UPDATE_SPINDLE_PWM); // Force update whenever updating block.
      #endif
    }
    
    // Initialize new segment
    segment_t *prep_segment = &segment_buffer[segment_buffer_head];

    // Set new segment to point to the current segment data block.
    prep_segment->st_block_index = prep.st_block_index;

    /*------------------------------------------------------------------------------------
        Compute the average velocity of this new segment by determining the total distance
      traveled over the segment time DT_SEGMENT. The following code first attempts to create
      a full segment based on the current ramp conditions. If the segment time is incomplete
      when terminating at a ramp state change, the code will continue to loop through the
      progressing ramp states to fill the remaining segment execution time. However, if
      an incomplete segment terminates at the end of the velocity profile, the segment is
      considered completed despite having a truncated execution time less than DT_SEGMENT.
        The velocity profile is always assumed to progress through the ramp sequence:
      acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
      may range from zero to the length of the block. Velocity profiles can end either at
      the end of planner block (typical) or mid-block at the end of a forced deceleration,
      such as from a feed hold.
    */
    #ifdef USER_DOUBLE
      double dt_max = DT_SEGMENT; // Maximum segment time
      double dt = 0.0; // Initialize segment time
      double time_var = dt_max; // Time worker variable
      double mm_var; // mm-Distance worker variable
      double speed_var; // Speed worker variable
      double mm_remaining = pl_block->millimeters; // New segment distance from end of block.
      double minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    #else
      float dt_max = DT_SEGMENT; // Maximum segment time
      float dt = 0.0; // Initialize segment time
      float time_var = dt_max; // Time worker variable
      float mm_var; // mm-Distance worker variable
      float speed_var; // Speed worker variable
      float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
      float minimum_mm = mm_remaining-prep.req_mm_increment; // Guarantee at least one step.
    #endif

    if (minimum_mm < 0.0) { minimum_mm = 0.0; }

    do {
      switch (prep.ramp_type) {
        case RAMP_DECEL_OVERRIDE:
          speed_var = pl_block->acceleration*time_var;
          if (prep.current_speed-prep.maximum_speed <= speed_var) {
            // Cruise or cruise-deceleration types only for deceleration override.
            mm_remaining = prep.accelerate_until;
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            prep.ramp_type = RAMP_CRUISE;
            prep.current_speed = prep.maximum_speed;
          } else { // Mid-deceleration override ramp.
            mm_remaining -= time_var*(prep.current_speed - 0.5*speed_var);
            prep.current_speed -= speed_var;
          }
          break;
        case RAMP_ACCEL:
          // NOTE: Acceleration ramp only computes during first do-while loop.
          speed_var = pl_block->acceleration*time_var;
          mm_remaining -= time_var*(prep.current_speed + 0.5*speed_var);
          if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
            // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
            mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
            time_var = 2.0*(pl_block->millimeters-mm_remaining)/(prep.current_speed+prep.maximum_speed);
            if (mm_remaining == prep.decelerate_after) { prep.ramp_type = RAMP_DECEL; }
            else { prep.ramp_type = RAMP_CRUISE; }
            prep.current_speed = prep.maximum_speed;
          } else { // Acceleration only.
            prep.current_speed += speed_var;
          }
          break;
        case RAMP_CRUISE:
          // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
          // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
          //   prevent this, simply enforce a minimum speed threshold in the planner.
          mm_var = mm_remaining - prep.maximum_speed*time_var;
          if (mm_var < prep.decelerate_after) { // End of cruise.
            // Cruise-deceleration junction or end of block.
            time_var = (mm_remaining - prep.decelerate_after)/prep.maximum_speed;
            mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
            prep.ramp_type = RAMP_DECEL;
          } else { // Cruising only.
            mm_remaining = mm_var;
          }
          break;
        default: // case RAMP_DECEL:
          // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
          speed_var = pl_block->acceleration*time_var; // Used as delta speed (mm/min)
          if (prep.current_speed > speed_var) { // Check if at or below zero speed.
            // Compute distance from end of segment to end of block.
            mm_var = mm_remaining - time_var*(prep.current_speed - 0.5*speed_var); // (mm)
            if (mm_var > prep.mm_complete) { // Typical case. In deceleration ramp.
              mm_remaining = mm_var;
              prep.current_speed -= speed_var;
              break; // Segment complete. Exit switch-case statement. Continue do-while loop.
            }
          }
          // Otherwise, at end of block or end of forced-deceleration.
          time_var = 2.0*(mm_remaining-prep.mm_complete)/(prep.current_speed+prep.exit_speed);
          mm_remaining = prep.mm_complete;
          prep.current_speed = prep.exit_speed;
      }
      dt += time_var; // Add computed ramp time to total segment time.
      if (dt < dt_max) { time_var = dt_max - dt; } // **Incomplete** At ramp junction.
      else {
        if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
          // Increase segment time to ensure at least one step in segment. Override and loop
          // through distance calculations until minimum_mm or mm_complete.
          dt_max += DT_SEGMENT;
          time_var = dt_max - dt;
        } else {
          break; // **Complete** Exit loop. Segment execution time maxed.
        }
      }
    } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

    #ifdef VARIABLE_SPINDLE
      /* -----------------------------------------------------------------------------------
        Compute spindle speed PWM output for step segment
      */
      
      if (st_prep_block->is_pwm_rate_adjusted || (sys.step_control & STEP_CONTROL_UPDATE_SPINDLE_PWM)) {
        if (pl_block->condition & (PL_COND_FLAG_SPINDLE_CW | PL_COND_FLAG_SPINDLE_CCW)) {
          #ifdef USER_DOUBLE
            double rpm = pl_block->spindle_speed;
          #else
            float rpm = pl_block->spindle_speed;
          #endif

          // NOTE: Feed and rapid overrides are independent of PWM value and do not alter laser power/rate.        
          if (st_prep_block->is_pwm_rate_adjusted) { rpm *= (prep.current_speed * prep.inv_rate); }
          // If current_speed is zero, then may need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE)
          // but this would be instantaneous only and during a motion. May not matter at all.
          prep.current_spindle_pwm = spindle_compute_pwm_value(rpm);
        } else { 
          sys.spindle_speed = 0.0;
          prep.current_spindle_pwm = SPINDLE_PWM_OFF_VALUE;
        }
        bit_false(sys.step_control,STEP_CONTROL_UPDATE_SPINDLE_PWM);
      }
      prep_segment->spindle_pwm = prep.current_spindle_pwm; // Reload segment PWM value

    #endif
    
    /* -----------------------------------------------------------------------------------
       Compute segment step rate, steps to execute, and apply necessary rate corrections.
       NOTE: Steps are computed by direct scalar conversion of the millimeter distance
       remaining in the block, rather than incrementally tallying the steps executed per
       segment. This helps in removing floating point round-off issues of several additions.
       However, since floats have only 7.2 significant digits, long moves with extremely
       high step counts can exceed the precision of floats, which can lead to lost steps.
       Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
       supported by Grbl (i.e. exceeding 10 meters axis travel at 200 step/mm).
    */
    #ifdef USER_DOUBLE
      double step_dist_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps
      double n_steps_remaining = ceil(step_dist_remaining); // Round-up current steps remaining
      double last_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining
    #else
      float step_dist_remaining = prep.step_per_mm*mm_remaining; // Convert mm_remaining to steps
      float n_steps_remaining = ceil(step_dist_remaining); // Round-up current steps remaining
      float last_n_steps_remaining = ceil(prep.steps_remaining); // Round-up last steps remaining
    #endif
    prep_segment->n_step = last_n_steps_remaining-n_steps_remaining; // Compute number of steps to execute.

    // Bail if we are at the end of a feed hold and don't have a step to execute.
    if (prep_segment->n_step == 0) {
      if (sys.step_control & STEP_CONTROL_EXECUTE_HOLD) {
        // Less than one step to decelerate to zero speed, but already very close. AMASS
        // requires full steps to execute. So, just bail.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Segment not generated, but current step data still retained.
      }
    }

    // Compute segment step rate. Since steps are integers and mm distances traveled are not,
    // the end of every segment can have a partial step of varying magnitudes that are not
    // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
    // compensate, we track the time to execute the previous segment's partial step and simply
    // apply it with the partial step distance to the current segment, so that it minutely
    // adjusts the whole segment rate to keep step output exact. These rate adjustments are
    // typically very small and do not adversely effect performance, but ensures that Grbl
    // outputs the exact acceleration and velocity profiles as computed by the planner.
    dt += prep.dt_remainder; // Apply previous segment partial step execute time
    #ifdef USER_DOUBLE
      double inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse
    #else
      float inv_rate = dt/(last_n_steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse
    #endif

    // Compute CPU cycles per step for the prepped segment.
    #ifdef USER_STM32
      uint32_t cycles = ceil(inv_rate*TICKS_PER_SECOND_60); // (cycles/step)
    #else
      uint32_t cycles = ceil( (TICKS_PER_MICROSECOND*1000000*60)*inv_rate ); // (cycles/step)
    #endif//USER_STM32

    #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
      // Compute step timing and multi-axis smoothing level.
      // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
      if (cycles < AMASS_LEVEL1) { prep_segment->amass_level = 0; }
      else {
        if (cycles < AMASS_LEVEL2) { prep_segment->amass_level = 1; }
        else if (cycles < AMASS_LEVEL3) { prep_segment->amass_level = 2; }
        else { prep_segment->amass_level = 3; }
        cycles >>= prep_segment->amass_level;
        prep_segment->n_step <<= prep_segment->amass_level;
      }
      if (cycles < (1UL << 16)) { prep_segment->cycles_per_tick = cycles; } // < 65536 (4.1ms @ 16MHz)
      else { prep_segment->cycles_per_tick = 0xffff; } // Just set the slowest speed possible.
    #else
      #ifdef USER_STM32
        // Compute step timing and timer prescalar for normal step generation.
        #ifdef USER_MANY_PRESCALER  //  (2^16 / 16Mhz) = 4.096ms
          if (cycles < (1UL << 16))
          { // < 2^16  (4.1ms @ 16MHz)
            prep_segment->prescaler = 1; //1 << 0 prescaler: 0
            prep_segment->cycles_per_tick = cycles;
          }
          else if (cycles < (1UL << 17))
          { // < 2^17  (8.2ms @ 16MHz)
            prep_segment->prescaler = 1 << 1; // prescaler: 2
            prep_segment->cycles_per_tick = cycles >> 1;
          }
          else if (cycles < (1UL << 18))
          { // < 2^18  (16.4ms @ 16MHz)
            prep_segment->prescaler = 1 << 2; // prescaler: 4
            prep_segment->cycles_per_tick = cycles >> 2;
          }
          else if (cycles < (1UL << 19))
          { // < 2^19 (32.8ms@16MHz)
            prep_segment->prescaler = 1 << 3; // prescaler: 8
            prep_segment->cycles_per_tick = cycles >> 3;
          }
          else if (cycles < (1UL << 20))
          { // < 2^20  (65.6ms @ 16MHz)
            prep_segment->prescaler = 1 << 4; // prescaler: 16
            prep_segment->cycles_per_tick = cycles >> 4;
          }
          else if (cycles < (1UL << 21))
          { // < 2^21  (131ms @ 16MHz)
            prep_segment->prescaler = 1 << 5; // prescaler: 32
            prep_segment->cycles_per_tick = cycles >> 5;
          }
//          else{
//            if (cycles < (1UL << 22))
//            { // < 2^22  (262ms @ 16MHz)
//              prep_segment->prescaler = 1 << 6; // prescaler: 64
//              prep_segment->cycles_per_tick = cycles >> 6;
//            }
//            else // Just set the slowest speed possible. (Around 4 step/sec.)
//            {
//              prep_segment->prescaler = 1 << 6; // prescaler: 64
//              prep_segment->cycles_per_tick = 0xffff;
//            }
//          }
          else if (cycles < (1UL << 22))
          { // < 2^22  (262ms @ 16MHz)
            prep_segment->prescaler = 1 << 6; // prescaler: 64
            prep_segment->cycles_per_tick = cycles >> 6;
          }
          else if (cycles < (1UL << 23))
          { // < 2^23  (524ms @ 16MHz)
            prep_segment->prescaler = 1 << 7; // prescaler: 128
            prep_segment->cycles_per_tick = cycles >> 7;
          }
          else if (cycles < (1UL << 24))
          { // < 2^24  (1048ms @ 16MHz)
            prep_segment->prescaler = 1 << 8; // prescaler: 256
            prep_segment->cycles_per_tick = cycles >> 8;
          }
          else
          {
            prep_segment->prescaler = 1 << 9; // prescaler: 512
            // < 2^25  (2096ms @ 16MHz)
            // Just set the slowest speed possible. (Around 1/2 step/sec.)
            prep_segment->cycles_per_tick = cycles >> 9;
          }
        #else // not USER_MANY_PRESCALER
          if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
            prep_segment->prescaler = 1; //1 << 0 prescaler: 0
            prep_segment->cycles_per_tick = cycles;
          } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
            prep_segment->prescaler = 1 << 3; // prescaler: 8
            prep_segment->cycles_per_tick = cycles >> 3;
          } else {
            prep_segment->prescaler = 1 << 6; // prescaler: 64
            if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
              prep_segment->cycles_per_tick =  cycles >> 6;
            }
            else{//(cycles < (1UL << 25)) { // < 2 (262ms@16MHz)
              prep_segment->prescaler = 1 << 9; // prescaler: 512
              // Just set the slowest speed possible. (Around 1/2 step/sec.)
              prep_segment->cycles_per_tick = cycles >> 9;
            }
          }
        #endif //USER_MANY_PRESCALER
      #else//not USER_STM32
        if (cycles < (1UL << 16)) { // < 65536  (4.1ms @ 16MHz)
          prep_segment->prescaler = 1; // prescaler: 0
          prep_segment->cycles_per_tick = cycles;
        } else if (cycles < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
          prep_segment->prescaler = 2; // prescaler: 8
          prep_segment->cycles_per_tick = cycles >> 3;
        } else {
          prep_segment->prescaler = 3; // prescaler: 64
          if (cycles < (1UL << 22)) { // < 4194304 (262ms@16MHz)
            prep_segment->cycles_per_tick =  cycles >> 6;
          } else { // Just set the slowest speed possible. (Around 4 step/sec.)
            prep_segment->cycles_per_tick = 0xffff;
          }
        }
      #endif//not USER_STM32
    #endif//ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    #ifdef USER_STM32
      if(prep_segment->cycles_per_tick >= 1)
        prep_segment->cycles_per_tick -= 1;
    #endif
    // Segment complete! Increment segment buffer indices, so stepper ISR can immediately execute it.
    segment_buffer_head = segment_next_head;
    if ( ++segment_next_head == SEGMENT_BUFFER_SIZE ) { segment_next_head = 0; }

    // Update the appropriate planner and segment data.
    pl_block->millimeters = mm_remaining;
    prep.steps_remaining = n_steps_remaining;
    prep.dt_remainder = (n_steps_remaining - step_dist_remaining)*inv_rate;

    // Check for exit conditions and flag to load next planner block.
    if (mm_remaining == prep.mm_complete) {
      // End of planner block or forced-termination. No more distance to be executed.
      if (mm_remaining > 0.0) { // At end of forced-termination.
        // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
        // the segment queue, where realtime protocol will set new state upon receiving the
        // cycle stop flag from the ISR. Prep_segment is blocked until then.
        bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
        #ifdef PARKING_ENABLE
          if (!(prep.recalculate_flag & PREP_FLAG_PARKING)) { prep.recalculate_flag |= PREP_FLAG_HOLD_PARTIAL_BLOCK; }
        #endif
        return; // Bail!
      } else { // End of planner block
        // The planner block is complete. All steps are set to be executed in the segment buffer.
        if (sys.step_control & STEP_CONTROL_EXECUTE_SYS_MOTION) {
          bit_true(sys.step_control,STEP_CONTROL_END_MOTION);
          return;
        }
        pl_block = NULL; // Set pointer to indicate check and load next planner block.
        plan_discard_current_block();
      }
    }

  }
}


// Called by realtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds.
#ifdef USER_DOUBLE
  double st_get_realtime_rate()
#else
  float st_get_realtime_rate()
#endif
{
  if (sys.state & (STATE_CYCLE | STATE_HOMING | STATE_HOLD | STATE_JOG | STATE_SAFETY_DOOR)){
    return prep.current_speed;
  }
  return 0.0f;
}
