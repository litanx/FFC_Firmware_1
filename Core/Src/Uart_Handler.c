/**
  ******************************************************************************
  * @file    Uart_Handler.c
  * @author  Diego B
  * @date    Oct 2020
  * @brief   Simple UART Command Rx/Tx handler
  *			 It's configured to work with UART1 but can be
  *			 easily changed to any other. Care must be taken
  *			 with the DMA channels initialization in the MSP function
  ******************************************************************************
  */

#include "main.h"
#include "stdio.h"
#include <stdarg.h>
#include "string.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

#define UART1_BUF_LEN 500		/* Ring Buffer size */
#define  UART1_iPtr   (UART1_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx))  // UART1 input pointer

uint8_t UART1_Rx_Buf[UART1_BUF_LEN];    /* Main Ring Buffer */
uint8_t UART1_DMA_Ovrn = 0;   			/* Override flag	*/
uint16_t UART1_oPtr = 0;      			/* Output pointer 	*/


static uint8_t UART1_getChar();
static uint8_t UART1_DataAvailable();

/* EXAMPLE HOW TO USE THIS DRIVER */
/*
main(){

  UART1_Init();
	UART1_printf("Hello world!\n\r");

	while(1){

		UART1_Handler();      // Polling for UART1 Commands
	}
}
*/

__weak void UART1_Cmd_Callback(uint8_t* cmd, uint16_t len){

	/* Process your commands here */

	/*----------------------------*/
}


void UART1_Init(){

  HAL_UART_Receive_DMA(&huart1, (uint8_t *)UART1_Rx_Buf, UART1_BUF_LEN);  // Initializes DMA for UART1

}

/*@Brief Gets Commands from UART1 RingBuffer
 *
 */
void UART1_Handler(){

	static uint8_t UART1_CMD_Buff[300];	/* Command Handler Buffer */
	static uint16_t UART1_cmdPtr = 0;

	while(UART1_DataAvailable()){

		uint8_t aux = UART1_getChar();

		if(aux == '\n' || aux == '\r' ) { // End Command

				UART1_CMD_Buff[UART1_cmdPtr] = aux;
				UART1_Cmd_Callback(UART1_CMD_Buff, UART1_cmdPtr);
				UART1_cmdPtr=0;

		}else{             				// Command body

		UART1_CMD_Buff[UART1_cmdPtr] = aux;
		if (UART1_cmdPtr < 299) UART1_cmdPtr++;

		}
	}
}

void UART1_printf(const char *fmt, ...){

	static char tempBuff[256];

	//memset(tempBuff, 0, 256);
	va_list arg;

	va_start (arg, fmt);
	uint16_t len = vsprintf(tempBuff,fmt, arg);
	va_end (arg);

	//HAL_UART_Transmit(&huart1, (const uint8_t*)tempBuff, len, 0xff); /* Transmit over uart */
	HAL_UART_Transmit_IT(&huart1, (const uint8_t*)tempBuff, len); /* Transmit over uart */
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    if(huart == &huart1){
      UART1_DMA_Ovrn = 1;
    }
}

static uint8_t UART1_DataAvailable() {
  if (((UART1_oPtr < UART1_iPtr) && !UART1_DMA_Ovrn) || ((UART1_oPtr > UART1_iPtr) && UART1_DMA_Ovrn)) {
    return 1;
  } else if (UART1_DMA_Ovrn){
    UART1_oPtr = UART1_iPtr;
    UART1_DMA_Ovrn = 0;
    return 0;
  }
  return 0;
}

static uint8_t UART1_getChar() {
  uint8_t ch = 0;
  if (UART1_DataAvailable()) {
    ch = UART1_Rx_Buf[UART1_oPtr++];
    if (UART1_oPtr >= UART1_BUF_LEN) {
      UART1_oPtr = 0;
      UART1_DMA_Ovrn = 0;
    }
  }
  return ch;
}

