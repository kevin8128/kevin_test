/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
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
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>

uart2_struct_ stRASPBERRYPI_UART2;


int main(void)
{ 
	uint16_t ui16WarmingUp = 0;

	enCURRENT_TASK _e_CURRENT_TASK;


	for(ui16WarmingUp=0; ui16WarmingUp<10000; ui16WarmingUp++)
	{
		Delay_us(1);		
	}

	ui16WarmingUp = 0;


    GPIOClockEnable();          // All GPIO clock enable

#if(MCO_VERIFY == ENABLE)	        
    MCO_Init();				    // Check system clock 72MHz (2021-08-12)
#endif

    UART_OUT_LED_Init();

	WIEGAND_IN_LED_Init();


	// UART Interface setting
    FW_UART_Init();				// UART1 PC
    
    RASPBERRYPI_UART_Init();	// UART2 RASPBERRY PI


	//Remocon_EXTI_Config();		// PA5 Remocon input pin

	//Timer2init();				// Timer2 10us

        

	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		while (1);
	}


	// All leds turn off
	UART_OUT_LED_ON_OFF(LED_OFF);
	WIEGAND_IN_LED_ON_OFF(LED_OFF);

          
	IWDG_Setting();		// IWDG Watchdog setting


	_e_CURRENT_TASK = UART_OUT_LED_ON;

	while (1)
	{
		switch (_e_CURRENT_TASK)
		{
			case UART_OUT_LED_ON:
					UART_OUT_LED_ON_OFF(LED_ON);
					_e_CURRENT_TASK = WIEGAND_IN_LED_ON;
					break;

			case WIEGAND_IN_LED_ON:
					WIEGAND_IN_LED_ON_OFF(LED_ON);
					_e_CURRENT_TASK = FW_UART_SEND;
					break;

			case FW_UART_SEND:
					/*
					FW_UART_string("SPM Board Aging & UART Test : ");
					ConverterToPutchar(ui16WarmingUp);
					if(ui16WarmingUp++ > 60000)
						ui16WarmingUp = 0;
					FW_UART_string("\r\n");
					_e_CURRENT_TASK = UART_OUT_LED_OFF;
					*/
					UART_BYPASS_RASPI_to_FW();        
                    _e_CURRENT_TASK = UART_OUT_LED_OFF;
					break;

			case UART_OUT_LED_OFF:
					UART_OUT_LED_ON_OFF(LED_OFF);
					_e_CURRENT_TASK = WIEGAND_IN_LED_OFF;
					break;
			
			case WIEGAND_IN_LED_OFF:
					WIEGAND_IN_LED_ON_OFF(LED_OFF);
					_e_CURRENT_TASK = UART_OUT_LED_ON;
					break;
			
			default:
				break;
		}

		//Delay_ms(100);

		IWDG_ReloadCounter();  // IWDG (WATCHDOG TIMER) 1000ms       
    }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
