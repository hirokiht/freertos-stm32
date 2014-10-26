#include "stm32f429.h"

void init_led(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO C clock. */
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Set the LED pin state such that the LED is off.  The LED is connected
     * between power and the microcontroller pin, which makes it turn on when
     * the pin is low.
     */
    GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);

    /* Configure the LED pin as push-pull output. */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void init_button(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO A clock */
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Configure the button pin as a floating input. */
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void enable_button_interrupts(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the AFIO clock.  GPIO_EXTILineConfig sets registers in
     * the AFIO.
     */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    /* Connect EXTI Line 0 to the button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure the EXTI line to generate an interrupt when the button is
     * pressed.  The button pin is high when pressed, so it needs to trigger
     * when rising from low to high. */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void init_rs232(USART_TypeDef* usart, uint32_t BaudRate){
	GPIO_InitTypeDef GPIO_InitStructure = {.GPIO_Mode = GPIO_Mode_AF, .GPIO_PuPd = GPIO_PuPd_NOPULL, .GPIO_OType = GPIO_OType_PP, .GPIO_Speed = GPIO_High_Speed};
    USART_InitTypeDef USART_InitStructure = {.USART_BaudRate = BaudRate, .USART_WordLength = USART_WordLength_8b, .USART_StopBits = USART_StopBits_1, .USART_Parity = USART_Parity_No, .USART_HardwareFlowControl = USART_HardwareFlowControl_None, .USART_Mode = USART_Mode_Rx | USART_Mode_Tx};
    NVIC_InitTypeDef NVIC_InitStructure = {.NVIC_IRQChannelPreemptionPriority = 6, .NVIC_IRQChannelSubPriority = 0, .NVIC_IRQChannelCmd = ENABLE};

    if(usart == USART1){
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// tx/rx: PA9/PA10, PB6/PB7
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    }else if(usart == USART2){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// tx/rx: PA2/PA3, PD5/PD6
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    }else if(usart == USART3){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	// tx/rx: PB10/PB11, PC10/PC11, PD8/PD9
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    }else if(usart == UART4){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	// tx/rx: PA0/PA1, PC10/PC11
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    }else if(usart == UART5){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);	// tx/rx: PC12/PD2
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_Init(GPIOD, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    }else if(usart == USART6){
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	// tx/rx: PC6/PC7, PG14/PG9
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    }else if(usart == UART7){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	// tx/rx: PE8/PE7, PF7/PF6
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    }else if(usart == UART8){
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	// tx/rx: PE1/PE0
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_UART8);
		GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_UART8);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
    }else return;

    USART_Init(usart, &USART_InitStructure);
    USART_Cmd(usart, ENABLE);

    USART_ITConfig(usart, USART_IT_TXE, DISABLE);
    USART_ITConfig(usart, USART_IT_RXNE, ENABLE);

    NVIC_SetPriorityGrouping(3);	//Set the interrupt to the correct group so that the priority will be higher than configMAX_SYSCALL_INTERRUPT_PRIORITY
    NVIC_Init(&NVIC_InitStructure); //Enable the USARTn_IRQ in the NVIC module (so that the USARTn interrupt handler is enabled).
}
