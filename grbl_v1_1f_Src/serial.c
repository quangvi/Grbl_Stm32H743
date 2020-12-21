/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
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

#define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
#define TX_RING_BUFFER (TX_BUFFER_SIZE+1)

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
#ifdef USER_NON_VOLATILE
  uint8_t serial_rx_buffer_tail = 0;
#else
  volatile uint8_t serial_rx_buffer_tail = 0;
#endif
uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint8_t serial_tx_buffer_head = 0;
#ifdef USER_NON_VOLATILE
  uint8_t serial_tx_buffer_tail = 0;
#else
  volatile uint8_t serial_tx_buffer_tail = 0;
#endif

// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{
  uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
  if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
  return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}


void serial_init()
{
  // Set baud rate
  #ifdef USER_STM32
    huart3.Init.BaudRate = BAUD_RATE;//9600;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
      Error_Handler();
    }

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart3.Instance->CR3, USART_CR3_EIE);

    /* Enable the UART Parity Error and Data Register not empty Interrupts */
    SET_BIT(huart3.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);

  //  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  //  SET_BIT(huart3.Instance->CR3, USART_CR3_EIE);

  //  /* Enable the Data Register not empty Interrupts */
  //  SET_BIT(huart3.Instance->CR1, USART_CR1_RXNEIE);

  ////  /* Disable the UART Parity Error Interrupt */
  //  CLEAR_BIT(huart3.Instance->CR1, USART_CR1_PEIE);
	#ifdef STM32H743xx
	#else
	  huart6.Init.BaudRate = BAUD_RATE;//9600;
	  if (HAL_HalfDuplex_Init(&huart6) != HAL_OK)
	  {
	    Error_Handler();
	  }
    #endif
  #else //not USER_STM32
    #if BAUD_RATE < 57600
      uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
      UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
    #else
      uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
      UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
    #endif
    UBRR0H = UBRR0_value >> 8;
    UBRR0L = UBRR0_value;

    // enable rx, tx, and interrupt on complete reception of a byte
    UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

    // defaults to 8-bit, no parity, 1 stop bit
  #endif //USER_STM32
}


// Writes one byte to the TX serial buffer. Called by main program.
void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  #ifdef USER_STM32
    /* Enable the UART Transmit data register empty Interrupt */
	#ifdef STM32H743xx
      SET_BIT(huart3.Instance->CR1, USART_CR1_TXEIE);
	#else
      SET_BIT(huart6.Instance->CR1, USART_CR1_TXEIE);
	#endif
  #else //not USER_STM32
    UCSR0B |=  (1 << UDRIE0);
  #endif //USER_STM32
}


// Data Register Empty Interrupt handler
#ifdef USER_STM32
  void ISR_SERIAL_UDRE_TX(void)
#else //not USER_STM32
  ISR(SERIAL_UDRE)
#endif //USER_STM32
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  #ifdef USER_STM32
    #ifdef STM32H743xx
      huart3.Instance->TDR = serial_tx_buffer[tail];// & (uint16_t)0x01FFU);
    #else
      huart6.Instance->DR = serial_tx_buffer[tail];// & (uint8_t)0x00FFU);
    #endif
  #else //not USER_STM32
    UDR0 = serial_tx_buffer[tail];
  #endif //USER_STM32
  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  #ifdef USER_STM32
    if (tail == serial_tx_buffer_head) {

      /* Disable the UART Transmit Complete Interrupt */
	  #ifdef STM32H743xx
        CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TXEIE);
	  #else
        CLEAR_BIT(huart6.Instance->CR1, USART_CR1_TXEIE);
	  #endif
  //
  //      /* Enable the UART Transmit Complete Interrupt */
  //      SET_BIT(huart6.Instance->CR1, USART_CR1_TCIE);

  //      /* Disable the UART Transmit Complete Interrupt */
  //      CLEAR_BIT(huart6.Instance->CR1, USART_CR1_TCIE);
    }
  #else //not USER_STM32
    if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
  #endif //USER_STM32
}


// Fetches the first byte in the serial read buffer. Called by main program.
uint8_t serial_read()
{
  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
  if (serial_rx_buffer_head == tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = serial_rx_buffer[tail];

    tail++;
    if (tail == RX_RING_BUFFER) { tail = 0; }
    serial_rx_buffer_tail = tail;

    return data;
  }
}

#ifdef USER_STM32
  void ISR_SERIAL_RX(void)
  {
	#ifdef STM32H743xx
	  uint8_t data = (uint8_t) READ_REG(huart3.Instance->RDR);
	#else
      uint8_t data = (uint8_t)(huart3.Instance->DR);// & (uint8_t)0x00FFU);
    #endif
#else //not USER_STM32
  ISR(SERIAL_RX)
  {
    uint8_t data = UDR0;
#endif //USER_STM32
  uint8_t next_head;

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            #ifdef USER_STM32
              case CMD_DEBUG_REPORT: {cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); sei();} break;
            #else
              case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
            #endif
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          #ifdef USER_RAPID_OVR_HIGH
            case CMD_RAPID_OVR_HIGH: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_HIGH); break;
          #endif
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}


void serial_reset_read_buffer()
{
  serial_rx_buffer_tail = serial_rx_buffer_head;
}
