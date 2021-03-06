/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-June-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f429i_discovery.h"

#include "uart.h"
#include "filesystem.h"
#include "fio.h"
#include "clib.h"
#include "shell.h"
#include "host.h"

#include "cmsis_os.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId LEDThread1Handle, shellThread;

/* Private function prototypes -----------------------------------------------*/
static void LED_Thread1(void const *argument);
static void Error_Handler(void);
static void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */

void send_byte(char ch){
	UART_send((uint8_t *)&ch,1);
}

char recv_byte(){
//	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	char msg;
	UART_recv((uint8_t *)&msg,1);
	return msg;
}

static void command_prompt(void const *pvParameters){
	char buf[128];
	char *argv[20];
        char hint[] = USER_NAME "@" USER_NAME "-STM32:~$ ";

	fio_printf(1, "\rWelcome to FreeRTOS Shell\r\n");
	while(1){
        fio_printf(1, "%s", hint);
		fio_read(0, buf, 127);
	
		int n=parse_command(buf, argv);

		/* will return pointer to the command function */
		cmdfunc *fptr=do_command(argv[0]);
		if(fptr!=NULL)
			fptr(n, argv);
		else
			fio_printf(2, "\r\n\"%s\" command not found.\r\n", argv[0]);
	}

}

int main(void){
	/* STM32F4xx HAL library initialization:
	   - Configure the Flash prefetch, instruction and Data caches
	   - Configure the Systick to generate an interrupt each 1 msec
	   - Set NVIC Group Priority to 4
	   - Global MSP (MCU Support Package) initialization
	 */
	HAL_Init();  

	/* Initialize LEDs */
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED4);

	/* Configure the system clock to 180 Mhz */
	SystemClock_Config();
	if(UART_init(USART1,115200) != HAL_OK)
	Error_Handler();

	/* Thread 1 definition */
	osThreadDef(LED3, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	/* Start thread 1 */
	LEDThread1Handle = osThreadCreate (osThread(LED3), NULL);

	/* Shell Thread definition */
	osThreadDef(CLI, command_prompt, osPriorityNormal, 0, 512);
	/* Start Shell */
	shellThread = osThreadCreate (osThread(CLI), NULL);

	/* Start scheduler    */
	osKernelStart (NULL, NULL);

	/* We should never get here as control is now taken by the scheduler */
	for(;;);
}

/**
  * @brief  Toggle LED3 and LED4 thread
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void const *argument){
	uint32_t count = 0;
	(void) argument;

	for(;;){
		for (count = osKernelSysTick() + 5000 ; count >= osKernelSysTick() ; osDelay(200))	// Toggle LED3 every 200 ms for 5s
			BSP_LED_Toggle(LED3);
		BSP_LED_Off(LED3);
		for (count = osKernelSysTick() + 5000 ; count >= osKernelSysTick() ; osDelay(400))	// Toggle LED3 every 400 ms for 5s
		  BSP_LED_Toggle(LED3);
	}
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void){
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	/* Enable Power Control clock */
	__PWR_CLK_ENABLE();

	/* The voltage scaling allows optimizing the power consumption when the device is 
	 clocked below the maximum system frequency, to update the voltage scaling value 
	 regarding system frequency refer to product datasheet.  */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/* Activate the Over-Drive mode */
	HAL_PWREx_ActivateOverDrive();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

static void Error_Handler(void){
    /* Turn LED4 (RED) on */
    BSP_LED_On(LED4);
    while(1);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1);
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
