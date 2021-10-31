#include "stm32f4_functions.h"
#include "math.h"
#include "stdio.h"

//KeiluV5

struct __FILE { int handle; };

FILE __stdout;
FILE __stdin;


int fputc(int ch, FILE* f)
{
	ITM_SendChar(ch);
	return(ch);
}


//STM32CUBEIDE
/*
int _write(int file, char *ptr, int len)
{
	for (int DataIdx = 0; DataIdx < len; DataIdx++)
		ITM_SendChar(*ptr++);

	return len;
}*/


void Init()
{
	//----- RCC Initialization -----------

	// Turn on HSE bit
	// Clear CR register
	RCC->CR &= ~RCC_CR_HSEON_Msk;
	// Set HSE bit
	RCC->CR |= RCC_CR_HSEON_Msk;

	// waiting for HSE run
	while ((RCC->CR & RCC_CR_HSERDY_Msk) == 0) {};

	// Turn on latency for Flash memory (0 to 7 cycles)
	// Clear latency value
	FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;
	// Flash latency set to 5 cycles (prefered to 168MHz clock)
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS;
	// Turn on prefetch buffer
	FLASH->ACR |= FLASH_ACR_PRFTEN_Msk;


	// SET System Clock on AHB magistral
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk; // HCLK=SYSCLK (system clock not divided - AHB prescaler equals 1)

	// Set APB2 and APB1 magistrals clock at max. speed (84MHz and 42MHz, respectively)
	RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk; // Clear APB2 prescaler - divider equals 1.
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // Set APB2 divider by 2 (168MHz/2 = 84MHz)

	RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk; // Clear APB1 prescaler - divider equals 1.
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // Set APB1 divider by 4 (168MHz/4 = 42MHz)

	// PLL Configuration 
	// fvco = fclockin x (PLLN/PLLM)
	// fpllout = fvco/PLLP
	// fclockin = 8MHz (this is freq. of crystal oscillator)
	// We want to get 168MHz of output PLL clock speed, so fpllout = 168MHz

	// THIS IS MAX FREQUENCY
	//unsigned int PLLM = 8;
	//unsigned int PLLN = 336;
	//unsigned int PLLP = 2;
	//unsigned int PLLQ = 4;

	unsigned int PLLM = 8;
	unsigned int PLLN = 336;
	unsigned int PLLP = 2;
	unsigned int PLLQ = 4;

	RCC->PLLCFGR = PLLM | (PLLN << 6) | (((PLLP >> 1) - 1) << 16) | (PLLQ << 24) | (1 << 22);

	// Turn on PLL (PLLON is set) 
	RCC->CR |= RCC_CR_PLLON_Msk;
	// Wait for establish a PLL clock
	while ((RCC->CR & RCC_CR_PLLRDY_Msk) == 0) {};

	// Set a PLL output as a source for CLK system
	RCC->CFGR &= ~RCC_CFGR_SW_Msk; // SW bits are clear
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // Set PLL source = clock system (SW = 2)

	// Wait for switch establish
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {};
}

void Deinit()
{
	//------ RCC Deinitialization -------
	//Set HSION bit 
	//RCC->CR |=0x01;

	//The same, but using names
	RCC->CR |= RCC_CR_HSION_Msk;

	//Reset CFGR register
	RCC->CFGR = 0x00;

	//Reset HSEON, CSSON, PLLON bits
	//RCC->CR &=0xFEF6FFFF;

	//using names as well
	RCC->CR &= ~(RCC_CR_HSEON_Msk | RCC_CR_CSSON_Msk | RCC_CR_PLLON_Msk);
	//
	//Reset PLLCFGR register
	RCC->PLLCFGR = RCC_PLLCFGR_RST_VALUE;
	//Reset HSEBYP bit
	RCC->CR &= ~(RCC_CR_HSEBYP_Msk);
	// Disable all interrupts
	RCC->CIR = 0x00;
	//------------------------------------
}


void LEDSEnable()
{
	//enable D section of GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN_Msk;

	//////////////////////////BLUE////////////////////
	//turn on 15 pin as 01 (output)
	GPIOD->MODER |= (1U << GPIO_MODER_MODER15_Pos);

	//push-pull is default config
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT15_Msk;

	//pull down configuration on pin 15
	GPIOD->PUPDR |= (0x2U << GPIO_PUPDR_PUPD15_Pos);

	//speed register - highest speed
	GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED15_Pos);


	//////////////////////////RED//////////////////////
	GPIOD->MODER |= (1U << GPIO_MODER_MODER14_Pos);
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT14_Msk;
	GPIOD->PUPDR |= (0x2U << GPIO_PUPDR_PUPD14_Pos);
	GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED14_Pos);


	//////////////////////////ORANGE///////////////////
	GPIOD->MODER |= (1U << GPIO_MODER_MODER13_Pos);
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT13_Msk;
	GPIOD->PUPDR |= (0x2U << GPIO_PUPDR_PUPD13_Pos);
	GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED13_Pos);


	//////////////////////////GREEN////////////////////
	GPIOD->MODER |= (1U << GPIO_MODER_MODER12_Pos);
	GPIOD->OTYPER &= ~GPIO_OTYPER_OT12_Msk;
	GPIOD->PUPDR |= (0x2U << GPIO_PUPDR_PUPD12_Pos);
	GPIOD->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED12_Pos);
}


void LEDOff(char color)
{
	switch (color)
	{
	case 'b':
		//set blue LED to 0
		GPIOD->ODR &= ~GPIO_ODR_ODR_15;
		break;

	case 'r':
		GPIOD->ODR &= ~GPIO_ODR_ODR_14;
		break;

	case 'o':
		GPIOD->ODR &= ~GPIO_ODR_ODR_13;
		break;

	case 'g':
		GPIOD->ODR &= ~GPIO_ODR_ODR_12;
		break;
	}
}


void LEDOn(char color)
{
	switch (color)
	{
	case 'b':
		//set 1 on 15 pin output (LED)
		GPIOD->ODR |= GPIO_ODR_ODR_15;
		break;

	case 'r':
		GPIOD->ODR |= GPIO_ODR_ODR_14;
		break;

	case 'o':
		GPIOD->ODR |= GPIO_ODR_ODR_13;
		break;

	case 'g':
		GPIOD->ODR |= GPIO_ODR_ODR_12;
		break;
		//OR

		//atomic bit set/reset - very quick
		//GPIOD->BSRR |= GPIO_BSRR_BS15_Msk;
	}
}


void TIM6Enable(int prescaler, int count, int interrupt)
{
	if (count <= 0)
		count = 1; //ms

	//apb1 clock to tim6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN_Msk;

	//choosing prescaler     in freq/(presc + 1)  out:1kHz -> pre:42000 -> pre:41999
	TIM6->PSC = prescaler - 1;

	//value of counting register -> x -> (x - 1)
	TIM6->ARR = count - 1;

	if (interrupt)
	{
		//enable the interrupt for tim6
		TIM6->DIER |= TIM_DIER_UIE_Msk;

		//clear pending interrupts
		NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);

		//enable interrupt in nvic
		NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}

	//start counter
	TIM6->CR1 |= TIM_CR1_CEN_Msk;
}


//void TIM6_DAC_IRQHandler()
//{
//	TIM6->SR &= ~TIM_SR_UIF_Msk;
//}


void TIM1Enable(int prescaler, int count, int interrupt)
{
	if (count <= 0)
		count = 1;

	//apb2 clock to tim1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN_Msk;

	//choosing prescaler
	TIM1->PSC = prescaler - 1;

	//value of counting register
	TIM1->ARR = count - 1;


	if (interrupt == 1)
	{
		//enable the interrupt for tim1
		TIM1->DIER |= TIM_DIER_UIE_Msk;

		//clear pending interrupts
		NVIC_ClearPendingIRQ(TIM1_UP_TIM10_IRQn);

		//enable interrupt in nvic
		NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	}

	//start counter
	TIM1->CR1 |= TIM_CR1_CEN_Msk;
}


//void TIM1_UP_TIM10_IRQHandler()
//{
//	TIM1->SR &= ~TIM_SR_UIF_Msk;
//}


void PWMTim1(int duty)
{
	//make sure that pwm will be valid
	if (TIM1->ARR < duty || duty <= 0)
		return;

	//stop counter
	TIM1->CR1 &= ~TIM_CR1_CEN_Msk;

	//enable E section of GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN_Msk;

	//enable SYSCFG for interrupts management
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

	//select alternate function 1 for pin
	GPIOE->AFR[1] |= GPIO_AFRH_AFSEL14_0;

	//turn on E14pin as alternative function
	GPIOE->MODER |= GPIO_MODER_MODER14_1;

	//speed register - highest speed
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED14_Msk;

	//capture/compare enable on channel 4
	TIM1->CCER |= TIM_CCER_CC4E_Msk;

	//enable preload for auto-reload (to avoid undefined behaviour)
	TIM1->CR1 |= TIM_CR1_ARPE_Msk;

	//set output compare 4 mode to PWM mode 1 (active while cnt<ccr4) and enable preload
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE_Msk;

	//PWM freq = fclk/PSC/ARR
	//PWM duty = CCR4/ARR
	//write duty cycle value
	TIM1->CCR4 = duty - 1;

	//reinitialize the counter and generate an update of the registers
	TIM1->EGR |= TIM_EGR_UG_Msk;

	//enable main output
	TIM1->BDTR |= TIM_BDTR_MOE_Msk;

	//set update interrupt flag to 0 (it is set after setting the UG bit
	//TIM1->SR &= ~TIM_SR_UIF_Msk;

	//set capture/compare interrupt flag to 0
	//TIM1->SR &= ~TIM_SR_CC4IF_Msk;

	//start counter
	TIM1->CR1 |= TIM_CR1_CEN_Msk;
}


void PWMTim1Update(int duty)
{
	//wait for counter overflow
	while (TIM1->SR &= TIM_SR_UIF_Msk);

	//write duty cycle
	TIM1->CCR4 = duty - 1;
}

void GenerateSignal(float* signal, int f, int f_s, int n_s, int sig)
{
	switch (sig) //0 - sine, 1 - square, 2 - saw
	{
		case 0:
		{
			for (int i = 0; i < n_s; i++)
				signal[i] = rint(127 * sin(f * (2 * M_PI) * i / f_s) + 128);

			break;
		}

		case 1:
		{
			for (int i = 0; i < n_s; i++)
			{
				if (i < n_s / 4)
					signal[i] = 255;
				else if (i < n_s / 2)
					signal[i] = 0;
				else if (i < (3 * n_s) / 4)
					signal[i] = 255;
				else
					signal[i] = 0;
			}
			break;
		}


		case 2:
		{
			double s = 1.0 / (n_s / 4);
			for (int i = 0; i < n_s; i++)
			{
				if (i < n_s / 4)
					signal[i] = rint((i * s) * 255);
				else if (i < n_s / 2)
				{
					if (i == n_s / 4)
						signal[i] = 1 * 255;
					else
						signal[i] = rint(((n_s / 2) % i * s) * 255);
				}
				else if (i < (3 * n_s) / 4)
					signal[i] = rint((i - (n_s / 2)) * s * 255);
				else
				{
					if (i == (3 * n_s) / 4)
						signal[i] = 1 * 255;
					else
						signal[i] = rint(((n_s) % i * s) * 255);
				}
			}
			break;
		}
	}

}


void RNGEnable(int interrupt)
{

	//Enable AHB2 clock to RNG module
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN_Msk;

	if (interrupt == 1) //Enable interrupt at the end of conversion
		RNG->CR |= RNG_CR_IE_Msk;

	//Enable RNG module
	RNG->CR |= RNG_CR_RNGEN_Msk;

	//wait for end of conversion
	//while((RNG->SR & RNG_SR_DRDY_Msk) == 0) {};

	//Check for errors
	//if((RNG->SR & RNG_SR_SEIS_Msk) || (RNG->SR & RNG_SR_CEIS_Msk))
		//return 0;

	//int32_t data = RNG->DR;
}

void ADCEnable()
{
	//enable apb2 clock to adc1
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN_Msk;

	//enable ADC in continuous mode
	ADC1->CR2 |= ADC_CR2_CONT_Msk;

	//enable end of conversion selection
	ADC1->CR2 |= ADC_CR2_EOCS_Msk;

	//one channel in sequence - one conversion
	ADC1->SQR1 |= (0 << ADC_SQR1_L_Pos);

	//conversion in regular sequence
	ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos);

	//sample time for the channel - max
	ADC1->SMPR2 |= ADC_SMPR2_SMP0_Msk;

	//enable interrupt at the end of conversion
	ADC1->CR1 |= ADC_CR1_EOCIE_Msk;

	//enable interrupt from adc peripheral
	NVIC_EnableIRQ(ADC_IRQn);

	//turn on conversion
	ADC1->CR2 |= ADC_CR2_ADON_Msk;

	//start conversion
	ADC1->CR2 |= ADC_CR2_SWSTART_Msk;
}


//void ADC_IRQHandler()
//{
//	uint16_t val;
//	//end of conversion
//	if (ADC1->SR & ADC_SR_EOC_Msk)
//	{
//		val = ADC1->DR;
//	}
//	ADC1->SR &= ~ADC_SR_EOC_Msk;
//}

void GPIOAEnable(int pin, char mode, int interrupt)
{
	//enable A section of GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;

	//enable SYSCFG for interrupts management
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

	//input or output
	switch (mode)
	{
	case 'i':
		//turn on PApin as input
		GPIOA->MODER |= (0U << pin * 2);
		GPIOA->MODER &= ~(0x1U << (1 + pin * 2));

		//push-pull is default config
		GPIOA->OTYPER &= ~pin;

		//pull down configuration on pin pin
		GPIOA->PUPDR &= ~(0x1U << pin * 2);
		GPIOA->PUPDR &= ~(0x1U << (1 + pin * 2));
		GPIOA->PUPDR |= (0x2U << pin * 2);

		//speed register - highest speed
		GPIOA->OSPEEDR |= (3U << pin * 2);

		if (interrupt == 1)
		{
			//enable interrupt on system register (EXTIpin_PA is always 0)
			SYSCFG->EXTICR[pin / 4] |= SYSCFG_EXTICR1_EXTI0_PA;

			//interrupt on rising edge
			EXTI->RTSR |= (1U << pin);

			//interrupt mask on exti line pin
			EXTI->IMR |= (1U << pin);

			//making sure that good irq is enabled
			int irq = pin;
			if (pin >= 5 && pin <= 9)
				irq = 23;
			else if (pin > 9)
				irq = 40;

			//interrupt handle to nvic
			NVIC_EnableIRQ(irq);
		}
		break;


	case 'o':
		//turn on PApin as output
		GPIOA->MODER |= (1U << pin * 2);

		//push-pull is default config
		GPIOA->OTYPER &= ~pin;

		//pull down configuration on pin pin
		GPIOA->PUPDR |= (0x2U << pin * 2);

		//speed register - highest speed
		GPIOA->OSPEEDR |= (3U << pin * 2);

		break;

	}

}

void GPIOAOn(int pin)
{
	GPIOA->ODR |= (0x1U << pin);
}

void GPIOAOff(int pin)
{
	GPIOA->ODR &= (0x0U << pin);
}


void GPIOBEnable(int pin, char mode, int interrupt)
{
	//enable B section of GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN_Msk;

	//enable apb2 to SYSCFG for interrupts management
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

	//input or output
	switch (mode)
	{
	case 'i':
		//turn on PBpin as input
		GPIOB->MODER &= ~(0x1U << pin * 2);
		GPIOB->MODER &= ~(0x1U << (1 + pin * 2));

		//push-pull is default config
		GPIOB->OTYPER &= ~pin;

		//pull down configuration on pin pin
		GPIOB->PUPDR &= ~(0x1U << pin * 2);
		GPIOB->PUPDR &= ~(0x1U << (1 + pin * 2));
		GPIOB->PUPDR |= (0x2U << pin * 2);

		//speed register - highest speed
		GPIOB->OSPEEDR |= (3U << pin * 2);

		if (interrupt == 1)
		{
			//enable interrupt on system register (EXTIpin_PA is always 0)
			SYSCFG->EXTICR[pin / 4] |= (0x1U << ((pin % 4) * 4));

			//interrupt on rising edge
			EXTI->RTSR |= (1U << pin);

			//interrupt mask on exti line pin
			EXTI->IMR |= (1U << pin);

			//making sure that good irq is enabled
			int irq = pin + 6;
			if (pin >= 5 && pin <= 9)
				irq = 23;
			else if (pin > 9)
				irq = 40;

			//interrupt handle to nvic
			NVIC_EnableIRQ(irq);
		}
		break;


	case 'o':
		//turn on PApin as output
		GPIOB->MODER |= (1U << pin * 2);

		//push-pull is default config
		GPIOB->OTYPER &= ~pin;

		//pull down configuration on pin pin
		GPIOB->PUPDR |= (0x2U << pin * 2);

		//speed register - highest speed
		GPIOB->OSPEEDR |= (3U << pin * 2);

		break;

	}

}

void GPIOBOn(int pin)
{
	GPIOB->ODR |= (0x1U << pin);
}

void GPIOBOff(int pin)
{
	GPIOB->ODR &= (0x0U << pin);
}


void ButtonEnable(int interrupt)
{
	//enable A section of GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN_Msk;

	//enable SYSCFG for interrupts management
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN_Msk;

	//turn on PA0 as input
	GPIOA->MODER |= (0U << GPIO_MODER_MODER0_Pos);

	//push-pull is default config
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT0_Msk;

	//pull down configuration on pin 0
	GPIOA->PUPDR |= (0x2U << GPIO_PUPDR_PUPD0_Pos);

	//speed register - highest speed
	GPIOA->OSPEEDR |= (3U << GPIO_OSPEEDR_OSPEED0_Pos);

	if (interrupt == 1)
	{
		//enable interrupt on system register
		SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

		//interrupt on rising edge
		EXTI->RTSR |= EXTI_RTSR_TR0_Msk;

		//interrupt mask on exti line 0
		EXTI->IMR |= EXTI_IMR_MR0_Msk;

		//interrupt handle to nvic
		NVIC_EnableIRQ(EXTI0_IRQn);
	}
}


int ButtonPushed()
{
	int state = 0;

	//read button
	if ((GPIOA->IDR & GPIO_IDR_ID0_Msk))
		state = 1;

	return state;
}

//void EXTI0_IRQHandler()
//{
//	if(i==2)
//		i=0;
//	i++;
//	EXTI->PR |= EXTI_PR_PR0_Msk;
//}


void CountEnable(int time)
{
	switch (time)
	{
	case 10: //10ms
		SysTick->LOAD = 0x19A28;
		break;

	case 100: //100ms
		SysTick->LOAD = 0x100590;
		break;

	case 1000: //1000ms
		SysTick->LOAD = 0xA037A0;
		break;
	}
	//clean counter
	SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;

	//we divide our system clock by 8
	SysTick->CTRL &= ~SysTick_CTRL_CLKSOURCE_Msk;

	//enable counter                               !!!!and interrupt!!!!
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; //| SysTick_CTRL_TICKINT_Msk
}


int CountCheck()
{
	int state = 0;

	//check counter flag
	if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
	{

		//reset counter flag
		SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
		while (1)
			if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk))
			{
				SysTick->CTRL &= ~SysTick_CTRL_COUNTFLAG_Msk;
				break;
			}
		state = 1;
		//we can put some code to extend our waiting more - like some variable, second flag, etc
	}

	return state;
}

void PWM__Init()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	GPIOA->MODER &= ~(GPIO_MODER_MODER15);
	GPIOA->MODER |= (GPIO_MODER_MODER15_1);

	GPIOA->AFR[1] |= GPIO_AFRH_AFSEL15_0;
	//	GPIOC->AFR[1] |= GPIO_AFRH_AFRH1_1;

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 = 0x0000;
	//	TIM2->CR2 = 0x0000;
	TIM2->CCER = 0x0000;
	TIM2->PSC = (uint16_t)65536; // 50Hz line speed -> 1us resolution.

	TIM2->ARR = (uint16_t)2565; //1000 us -> 1kHz. Correction due to change on nth cycle

	TIM2->CR1 |= TIM_CR1_ARPE;

	TIM2->CCMR1 = 0x0000;
	//	TIM2->CCMR2 = 0x0000;

	TIM2->CCMR1 |= (0x06 << TIM_CCMR1_OC1M_Pos | TIM_CCMR1_OC1PE);
	//	TIM2->CCMR2 |= (0x06 << TIM_CCMR2_OC4M_Pos | TIM_CCMR2_OC4PE);

	TIM2->CCR2 = 0;
	TIM2->CCR4 = 0;

	TIM2->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC4E);

	TIM2->BDTR |= TIM_BDTR_MOE;
}

void PWM__Start()
{
	TIM2->ARR = 2565;
	TIM2->CCR1 = 1200;
	//	TIM3->CCR4 = 1;
	TIM2->CR1 |= TIM_CR1_CEN;
	// Ugly, but works for now.
//	delay_ms(500);
//	delay_ms(500);
//	delay_ms(500);
//	delay_ms(500);

//	TIM2->ARR = PWMpid.period;
//	PWM__Update(PWMpid.out_init);
}
