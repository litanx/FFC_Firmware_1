/**
  ******************************************************************************
  * @file    Uart_Handler.h
  * @author  Diego B
  * @date    Oct 2020
  * @brief   Simple UART Command Rx/Tx handler
  ******************************************************************************
  */

  #ifndef UART_HANDLER
  #define UART_HANDLER

void UART1_Init();
void UART1_Handler();
void UART1_printf(const char *fmt, ...);
void UART1_Cmd_Callback(uint8_t* cmd, uint16_t len);

#endif
