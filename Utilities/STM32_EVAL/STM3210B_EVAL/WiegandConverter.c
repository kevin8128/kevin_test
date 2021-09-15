#include "stm32f10x.h"
#include "stm32_eval.h"
#include <stdio.h>
#include "WiegandConverter.h"



uart1_struct_ stFW_UART;
uart2_struct_ stRASPBERRYPI_UART;


__IO uint32_t LsiFreq = 40000;		// IWDG internal internal RC frequency (40KHz)


/**
  * @brief  Delay microsecond  
  * @note   
  * @param  None
  * @retval None
  */
void Delay_us(uint16_t time_us)			/* time delay for us in 72MHz */
{
	register unsigned int i;

	for(i = 0; i < time_us; i++)
	{ 
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");
		asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP"); asm volatile("NOP");	      
	}
}

/**
  * @brief  Delay milisecond  
  * @note   
  * @param  None
  * @retval None
  */
void Delay_ms(uint16_t time_ms)			/* time delay for ms in 72MHz */
{
	register unsigned int i;

	for(i = 0; i < time_ms; i++)
		Delay_us(1000);
}




/**
  * @brief  All GPIO clock setting
  *         GPIOA
  *         GPIOB
  *         GPIOC
  * @note   
  * @param  None
  * @retval None
  */
void GPIOClockEnable(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 
}



/**
  * @brief  MCO Function
  * @note   PLL clock divided by 2 selected (72MHz / 2) = 36MHz
  *         RCC_MCO_HSE = Output external xtal value 8MHz
  * @param  None
  * @retval None
  */
void MCO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    //RCC_MCOConfig(RCC_MCO_HSE);                 // Output external xtal value 8MHz
    RCC_MCOConfig(RCC_MCO_PLLCLK_Div2);           // PLL clock divided by 2 selected (72MHz / 2) = 36MHz
}



/**
  * @brief  UART_LED FUNCTION
  * @note   GREEN color, PORTA12  
  * @param  None
  * @retval None
  */
void UART_OUT_LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIOA->BSRR = GPIO_Pin_12;
}


void UART_OUT_LED_ON_OFF(uint8_t led_status)
{
    if(led_status)
    {
        GPIOA->BSRR = GPIO_Pin_12;        
    }
    else
    {
        GPIOA->BRR = GPIO_Pin_12;
    }
}


/**
  * @brief  WIEGAND_IN LED FUNCTION
  * @note   RED color, PORTA11  
  * @param  None
  * @retval None
  */
void WIEGAND_IN_LED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
        
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIOA->BSRR = GPIO_Pin_11;
}


unsigned char tempBool = 1;
void WIEGAND_IN_LED_ON_OFF(uint8_t led_status)
{
    
    
    if(led_status)
    {
        GPIOA->BSRR = GPIO_Pin_11;
    }
    else
    {
        GPIOA->BRR = GPIO_Pin_11;
    }
}



/**
  * @brief  IWDG Watchdog timer
  * @note   1second = (1/40KHz) x 64 x 625
  * @param  None
  * @retval None
  */
void IWDG_Setting(void)
{
	/* IWDG timeout equal to 1000 ms (the timeout may varies due to LSI frequency dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/64 */
	IWDG_SetPrescaler(IWDG_Prescaler_64);

	/* Set counter reload value to obtain 1000ms IWDG TimeOut.
	 Counter Reload Value = 1000ms/IWDG counter clock period	                     
	*/
	IWDG_SetReload(LsiFreq/64);		// Reload value = 625

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}


/*
void Remocon_EXTI_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	EXTI_InitTypeDef  EXTI_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				// PORTA5 remocon input pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);

	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	

	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}



void EXTI9_5_IRQHandler(void)
{
	uint8_t x;
	
	if(EXTI_GetITStatus(EXTI_Line5) != RESET)
	{			
		EXTI_ClearITPendingBit(EXTI_Line5);

		Remocon_count = TIM_GetCounter(TIM2);		// Save Tim2 counter value
		TIM_SetCounter(TIM2, 0);					// Clear Tim2 counter register

		switch(Remocon_state)
	    { 
			case 0 :					// if ready state, go to step 1(leader state)
	               Remocon_state = 1;
	               Remocon_count = 0;           
	               break;
			case 1 : 					// if leader state, check leader signal
	               if((Remocon_count >= (Leader_Length - (Leader_Length * IR_Tolerance)) ) && (Remocon_count <= (Leader_Length + (Leader_Length * IR_Tolerance))  ))
	                 { 
					 	Remocon_state = 2;		// if lead signal, go to step 2(data state)
	                    Remocon_data = 0;
		                Remocon_bit_count = 0;
	                 }
	               else
				   		Remocon_state = 0;		// if not lead signal, go to step 0(ready state)
		       	   break;

	         case 2: 					// if data state, check data signal 0 or 1
			       Remocon_data >>= 1;
	               if((Remocon_count >= (Low_Bit_Length - (Low_Bit_Length * IR_Tolerance))) && (Remocon_count <= (Low_Bit_Length + (Low_Bit_Length * IR_Tolerance))))
	                 	Remocon_data &= 0x7F;		// if data 0, add data bit 0
	               else if((Remocon_count >= (High_Bit_Length - (High_Bit_Length * IR_Tolerance))) && (Remocon_count <= (High_Bit_Length + (High_Bit_Length * IR_Tolerance))))
	                 	Remocon_data |= 0x80;
	               else
		         		Remocon_state = 0;		// if not 0 and not 1, go to step 0(ready state)
	               Remocon_bit_count++;

			   	   if((Remocon_bit_count % 8) == 0) // if a character complete, store it
		           {
				       x = Remocon_data;
			   		   Remocon_command[(Remocon_bit_count/8)-1] = x;
		               Remocon_data = 0;
	               }

				   if(Remocon_bit_count == 32)	// if remocon OK, check custom code and checksum
	               { 
				       x = Remocon_command[3];
			           if((~Remocon_command[2] & 0xFF) == x)
			           { 
					   	   switch(Remocon_command[2])		// if position key = repeat, else only one
					   	   {
						   		case 0x02:
								case 0x07:
								case 0x09:
								case 0x0E: Remocon_state = 3;
								           Remocon_Repeat_Toggle_count = 0;
									break;
								default: Remocon_state = 0;
								    break;
					   	   }
					       
			               Remocon_OK_flag = 1;
			           }
	                   else
			               Remocon_state = 0;
		 	       }
                   break;
			 case 3:
			 	   if( (Remocon_count >= FIRST_Message_Min) && (Remocon_count <= FIRST_Message_Max))
			 	   {
				   		Remocon_state = 4;
			 	   }
				   else
				   {
				   		Remocon_state = 0;
						Remocon_count = 0;
				   }				   
			 	break;
			case 4:
					if((Remocon_count >= IR_Repeat_Min) && (Remocon_count <= IR_Repeat_Max))
					{
						Remocon_state = 4;
						
						if(Remocon_Repeat_Toggle_count > 1)		// Delay effect (abot 320ms)
						{
							Remocon_Repeat_Toggle_count = 0;
							Remocon_OK_flag = 1;
						}
						else
						{
							Remocon_Repeat_Toggle_count++;
						}
					}
                    else if((Remocon_count >= 1063) && (Remocon_count <= 1298))
                    {
						Remocon_state = 4;
                    }
					else
					{
						Remocon_state = 0;
						Remocon_count = 0;
						Remocon_Repeat_Toggle_count = 0;
					}
				break;
				   	

	        default : break;
	    }
	}
}
*/

/**********************************************************************************************************************************
                                                       Timer FUNCTIONS
**********************************************************************************************************************************/
/*
void Timer2init(void)		// TIM2
{
	NVIC_InitTypeDef   NVIC_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	TIM_TimeBaseStructure.TIM_Period = ARR_value;			// 0.00001 * 12900 = 129ms
	TIM_TimeBaseStructure.TIM_Prescaler = 720-1;			// 720 / 72MHz = 0.00001 (10us)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM2, TIM_IT_Update,ENABLE);	
}


void TIM2_IRQHandler(void)
{	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	
	Remocon_state = 0;
}
*/





/**
  * @brief  FW_UART_Init
  * @note   Baudrate 19200bps
  * @param  None
  * @retval None
  */
void FW_UART_Init(void)					// UART1
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStruct;
    NVIC_InitTypeDef   NVIC_InitStruct;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
          
    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                   /*UART1_TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                   /*UART1_RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStruct.USART_BaudRate  = 19200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits  = USART_StopBits_1;
    USART_InitStruct.USART_Parity  = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode  = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);
    
    USART_Cmd(USART1, ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}



void USART1_IRQHandler(void)            // PC FW UART1 Receive interrupt
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {        
        stFW_UART.BUFFER[stFW_UART.HEAD++] = USART1->DR;
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);       
                
        if (stFW_UART.HEAD == PC_RX_BUFFER_SIZE) stFW_UART.HEAD = 0;            
    }
}



void FW_UART_putchar(uint8_t data)
{
    USART_SendData(USART1,data);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
}



void FW_UART_string(uint8_t *string)
{
    while (*string != '\0')
    {
        FW_UART_putchar(*string++);
    }
}


void FW_UART_LOOPTEST(void)
{
	if(stFW_UART.HEAD != stFW_UART.TAIL)
	{
		FW_UART_putchar(stFW_UART.BUFFER[stFW_UART.TAIL++]);
        if (stFW_UART.TAIL == RASPBERRYPI_RX_BUFFER_SIZE) stFW_UART.TAIL = 0;    
	}
}


void ConverterToPutchar(uint16_t value)
{
	uint16_t tempValue = 0;

	if(value / 10000)	FW_UART_putchar( (value/10000) + '0');
	else                FW_UART_putchar('0');
	
	tempValue = value % 10000;

	if(tempValue / 1000) FW_UART_putchar( (tempValue / 1000) + '0');
	else                 FW_UART_putchar('0');

	tempValue = tempValue % 1000;

	if(tempValue / 100) FW_UART_putchar( (tempValue / 100) + '0');
	else                FW_UART_putchar('0');

	tempValue = tempValue % 100;

	if(tempValue / 10) FW_UART_putchar( (tempValue / 10) + '0');
	else               FW_UART_putchar('0');

	FW_UART_putchar( (tempValue % 10) + '0');
}




/**
  * @brief  RASPBERRYPI_UART_Init
  * @note   Baudrate 19200bps
  * @param  None
  * @retval None
  */
void RASPBERRYPI_UART_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStruct;
    NVIC_InitTypeDef   NVIC_InitStruct;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
         
    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                   /*UART2_TX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                   /*UART2_RX*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStruct.USART_BaudRate  = 19200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits  = USART_StopBits_1;
    USART_InitStruct.USART_Parity  = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode  = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStruct);
    
    USART_Cmd(USART2, ENABLE);
    
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}



void USART2_IRQHandler(void)            // RASPBERRYPI_UART Receive interrupt
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {		
        stRASPBERRYPI_UART.BUFFER[stRASPBERRYPI_UART.HEAD++] = USART2->DR;
		        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);  
                
        if (stRASPBERRYPI_UART.HEAD == RASPBERRYPI_RX_BUFFER_SIZE) stRASPBERRYPI_UART.HEAD = 0;         
    }
}



void RASPBERRYPI_UART_putchar(uint8_t data)
{
    USART_SendData(USART2,data);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE)==RESET);
}



void RASPBERRYPI_UART_string(uint8_t *string)
{
    while (*string != '\0')
    {
        RASPBERRYPI_UART_putchar(*string++);
    }
}


void UART_BYPASS_RASPI_to_FW(void)
{
	if(stRASPBERRYPI_UART.HEAD != stRASPBERRYPI_UART.TAIL)
	{
		FW_UART_putchar(stRASPBERRYPI_UART.BUFFER[stRASPBERRYPI_UART.TAIL++]);
		if (stRASPBERRYPI_UART.TAIL == RASPBERRYPI_RX_BUFFER_SIZE)
                  stRASPBERRYPI_UART.TAIL = 0;    
                
	}
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
