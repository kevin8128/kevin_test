/**
  ******************************************************************************
  * @file    stm3210c_eval.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file contains definitions for STM3210C_EVAL's Leds, push-buttons
  *          COM ports, SD Card on SPI and sEE on I2C hardware resources.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 210 STMicroelectronics</center></h2>
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210C_EVAL_H
#define __STM3210C_EVAL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32_eval.h"

#define R4K_DEBUG_STRING   0
#define MCO_VERIFY         DISABLE


#define PC_RX_BUFFER_SIZE              1000          // PC  UART1 receive buffer size
#define RASPBERRYPI_RX_BUFFER_SIZE     1000          // RASPBERRYPI  UART2 receive buffer size

#define ON_STATUS    1
#define OFF_STATUS   0

#define HIGH         1
#define LOW          0

#define LED_ON       1
#define LED_OFF      0
   
typedef enum {
    UART_OUT_LED_ON = 0,
    WIEGAND_IN_LED_ON,
    FW_UART_SEND,
    UART_OUT_LED_OFF,
    WIEGAND_IN_LED_OFF
} enCURRENT_TASK;


typedef struct struct_remocon_toggle
{
	unsigned char SetIdNum;
	unsigned char PowerState;
	unsigned char MuteState;
	unsigned char MenuDisplayState;
	unsigned char KeyLockState;	
}remocon_toggle_;


typedef struct struct_uart1_ring			 // FW UART1
{
    unsigned int HEAD;
    unsigned int TAIL;
    unsigned int BUFFER[PC_RX_BUFFER_SIZE];
}uart1_struct_;         


typedef struct struct_uart2_ring			 // RASPBERRY PI UART2
{
    unsigned int HEAD;
    unsigned int TAIL;
    unsigned int BUFFER[RASPBERRYPI_RX_BUFFER_SIZE];
}uart2_struct_;     


void Delay_us(uint16_t time_us);			/* time delay for us in 72MHz */
void Delay_ms(uint16_t time_ms);			/* time delay for ms in 72MHz */



////////////////////////////// Clock Setting ///////////////////////////////////
void GPIOClockEnable(void);
void MCO_Init(void);



///////////////////////////// LED DISPLAY /////////////////////////////////////
void UART_OUT_LED_Init(void);                   // GREEN
void UART_OUT_LED_ON_OFF(uint8_t led_status);
void WIEGAND_IN_LED_Init(void);             // RED
void WIEGAND_IN_LED_ON_OFF(uint8_t led_status);



///////////////////////////// Watchdog Timer setting /////////////////////////
void IWDG_Setting(void);



///////////////////////////// External interrypt setting ////////////////////
void Remocon_EXTI_Config(void);
void EXTI9_5_IRQHandler(void);



///////////////////////////// TIM2 functions  /////////////////////////////////
void Timer2init(void);
void TIM2_IRQHandler(void);



///////////////////////////// FW UART1 PC functions ///////////////////////////
void FW_UART_Init(void);
void FW_UART_putchar(uint8_t data);
void FW_UART_string(uint8_t *string);
void FW_USART1_IRQHandler(void);            // PC UART1 RX interrupt
void ConverterToPutchar(uint16_t value);
void FW_UART_LOOPTEST(void);

////////////////////// RASPBERRYPI UART2 PC functions /////////////////////////
void RASPBERRYPI_UART_Init(void);
void RASPBERRYPI_UART_putchar(uint8_t data);
void RASPBERRYPI_UART_string(uint8_t *string);
void RASPBERRYPI_IRQHandler(void);			   // RASPBERRYPI UART2 RX interrupt
void UART_BYPASS_RASPI_to_FW(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM3210C_EVAL_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */
    
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
