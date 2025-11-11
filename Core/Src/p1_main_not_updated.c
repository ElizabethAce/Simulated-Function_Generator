///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2024 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//
//#include "main.h"
//#include "math.h"
//
//void SystemClock_Config(void);
//void TIM2_init(void);
//void DAC_init(void);
//void DAC_write(uint16_t in_val);
//uint16_t DAC_volt_conv(uint16_t in_val);
//int32_t keypad_read(void);
//int32_t read_rows(void);
//int32_t find_key(int32_t row_indx, int32_t col);
//
//
//
//int main(void)
//{
//  HAL_Init();
//
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//
//  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN); 			// enables GPIOA clock used for DAC
//  RCC->AHB2ENR   |= RCC_AHB2ENR_GPIOBEN;				// enable GPIOB Clock for ports on board rows/cols
//  RCC->AHB2ENR	 |= RCC_AHB2ENR_GPIOCEN;				// enable GPIOC Clock for LEDS
//
//  TIM2_init();
//  DAC_init();
//
//
//
//  // Configure LED pins
//  GPIOC->MODER	&= ~(GPIO_MODER_MODE0_Msk);	// output mode
//  GPIOC->MODER |= (1 << GPIO_MODER_MODE0_Pos);	// mode to 01
//  GPIOC->OTYPER	&= ~(GPIO_OTYPER_OT0_Msk);	// push-pull output
//  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk);	//low speed
//  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk);	//no resistor
//
//  GPIOC->MODER	&= ~(GPIO_MODER_MODE1_Msk);	// output mode
//  GPIOC->MODER |= (1 << GPIO_MODER_MODE1_Pos);	// mode to 01
//  GPIOC->OTYPER	&= ~(GPIO_OTYPER_OT1_Msk);	// push-pull output
//  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED1_Msk);	//low speed
//  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD1_Msk);	//no resistor
//
//  GPIOC->MODER	&= ~(GPIO_MODER_MODE2_Msk);	// output mode
//  GPIOC->MODER |= (1 << GPIO_MODER_MODE2_Pos);	// mode to 01
//  GPIOC->OTYPER	&= ~(GPIO_OTYPER_OT2_Msk);	// push-pull output
//  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED2_Msk);	//low speed
//  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD2_Msk);	//no resistor
//
//  GPIOC->MODER	&= ~(GPIO_MODER_MODE3_Msk);	// output mode
//  GPIOC->MODER |= (1 << GPIO_MODER_MODE3_Pos);	// mode to 01
//  GPIOC->OTYPER	&= ~(GPIO_OTYPER_OT3_Msk);	// push-pull output
//  GPIOC->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED3_Msk);	//low speed
//  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD3_Msk);	//no resistor
//
//
//  // Configure pins for rows to read
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE0_Msk);	// prepare to read row 0
//  GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD0_Msk);	//
//  GPIOB->PUPDR  |= (2 << GPIO_PUPDR_PUPD0_Pos);	// pull-down
//
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE1_Msk);	// prepare to read row 1
//  GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD1_Msk);	//
//  GPIOB->PUPDR  |= (2 << GPIO_PUPDR_PUPD1_Pos);	// pull-down
//
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE2_Msk);	// prepare to read row 2
//  GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD2_Msk);	//
//  GPIOB->PUPDR  |= (2 << GPIO_PUPDR_PUPD2_Pos);	// pull-down
//
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE3_Msk);	// prepare to read row 3
//  GPIOB->PUPDR  &= ~(GPIO_PUPDR_PUPD3_Msk);	//
//  GPIOB->PUPDR  |= (2 << GPIO_PUPDR_PUPD3_Pos);	// pull-down
//
//  // Configure pins for columns
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE4_Msk);	// output mode
//  GPIOB->MODER |= (1 << GPIO_MODER_MODE4_Pos);	// mode to 01
//  GPIOB->OTYPER	&= ~(GPIO_OTYPER_OT4_Msk);	// push-pull output
//  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4_Msk);	//low speed
//  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4_Msk);	//no resistor
//
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE5_Msk);	// output mode
//  GPIOB->MODER |= (1 << GPIO_MODER_MODE5_Pos);	// mode to 01
//  GPIOB->OTYPER	&= ~(GPIO_OTYPER_OT5_Msk);	// push-pull output
//  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_Msk);	//low speed
//  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);	//no resistor
//
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE6_Msk);	// output mode
//  GPIOB->MODER |= (1 << GPIO_MODER_MODE6_Pos);	// mode to 01
//  GPIOB->OTYPER	&= ~(GPIO_OTYPER_OT6_Msk);	// push-pull output
//  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED6_Msk);	//low speed
//  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk);	//no push/pull resistor
//
//  GPIOB->MODER	&= ~(GPIO_MODER_MODE7_Msk);	// output mode
//  GPIOB->MODER |= (1 << GPIO_MODER_MODE7_Pos);	// mode to 01
//  GPIOB->OTYPER	&= ~(GPIO_OTYPER_OT7_Msk);	// push-pull output
//  GPIOB->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7_Msk);	//low speed
//  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);	//no push/pull resistor
//
//  // sets all columns high
//  //GPIOB->BSRR |= (col_msk);
//  GPIOB->BSRR |= (1 << GPIO_BSRR_BS4_Pos);	// sets first column high
//  GPIOB->BSRR |= (1 << GPIO_BSRR_BS5_Pos);	// " "  second " "   " "
//  GPIOB->BSRR |= (1 << GPIO_BSRR_BS6_Pos);	// " "  third  " "   " "
//  GPIOB->BSRR |= (1 << GPIO_BSRR_BS7_Pos);	// " "  fourth " "   " "
//
//  GPIOC->ODR &= (0);	// clear LEDS
//
//
//
//
//  while (1)
//  {
//
//	int32_t disp_key = keypad_read();			// disp_key is holding my key number 0-15
//
//	if (disp_key != -1){						// key was pressed
//		  GPIOC->ODR &= (0);					// clear output register for LEDS
//
//		  GPIOC->BSRR |= (disp_key);			// displays key on LEDS
//
//		  for (int i = 0; i < 70000; i++);		// delay so it can take a key at a time
//	}
//
//	//DAC_write(DAC_volt_conv(volt));
//
//
//	GPIOB->BSRR |= (1 << GPIO_BSRR_BS4_Pos);	// sets first column high
//	GPIOB->BSRR |= (1 << GPIO_BSRR_BS5_Pos);	// " "  second " "   " "
//	GPIOB->BSRR |= (1 << GPIO_BSRR_BS6_Pos);	// " "  third  " "   " "
//	GPIOB->BSRR |= (1 << GPIO_BSRR_BS7_Pos);	// " "  fourth " "   " "
//  }
//
//
//}
//
//void TIM2_init(void)
//{
//	// TIM2 Configuration
//	RCC->APB1ENR1  |= (RCC_APB1ENR1_TIM2EN);		// TIM2 clock enable
//
//	// Configure output on Port A, pin 1
//	GPIOA->MODER		&= ~(GPIO_MODER_MODE1_Msk); 	// Resets PC0 MODER bits to 0 (clear)
//	GPIOA->MODER		|= (1<<GPIO_MODER_MODE1_Pos); 	// Set PC0 MODER bits to 01 -> Output MODE
//	GPIOA->OTYPER		&= ~(GPIO_OTYPER_OT1_Msk); 		// OTYPE in push-pull
//	GPIOA->OSPEEDR		&= ~(GPIO_OSPEEDR_OSPEED1_Msk); // Slow speed
//	GPIOA->PUPDR		&= ~(GPIO_PUPDR_PUPD1_Msk); 	// No resistor (bits set to 0)
//
//	TIM2->PSC = 10;									// 25
//	TIM2->ARR = 72;								// 4000
//	TIM2->CCR1 = 17;								// 400 (4MHz/5kHz)/2 = 400 , divide by 2 to get the 50% duty cycle/ON half of a cycle
//	// TIM2 Control Register Configuration
//	TIM2->CR1 = 0;									// Default timer settings
//	TIM2->CR1 &= ~(TIM_CR1_DIR); 						// Sets counter direction up, (0: counts up)
//	TIM2->CR1 |= (TIM_CR1_URS); 						// Set to 1 --> Only overflow/underflow generates an update interrupt
//	TIM2->CR1 &= ~(TIM_CR1_UDIS); 					// Enable update events (UEV), bit cleared to 0
//	// Enabling
//	TIM2->DIER |= TIM_DIER_UIE; 						// Enable update interrupt (bit 0 set to 1)
//	TIM2->DIER |= TIM_DIER_CC1IE;						// Enable CCR1 (bit 1 set to 1)
//	NVIC->ISER[0] |= (1<<TIM2_IRQn);					// Enable TIM2 interrupt in the NVIC
//	TIM2->CR1 |= TIM_CR1_CEN; 						// Counter enabled, timer starts, (bit 0 is set to 1)
//
//}
//
//void DAC_init(void)
//{
//	RCC->APB2ENR   |=  (RCC_APB2ENR_SPI1EN);			// SPI clock enable
//
//	// Configure chip select pin
//	GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk);
//	GPIOA->MODER |= (1<<GPIO_MODER_MODE0_Pos);
//	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0_Msk);
//	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk);
//	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk);
//
//	GPIOA->BSRR |= (1<<GPIO_BSRR_BS0_Pos);				// CS high when not being used
//
//	// Configure VDD and VRef
//		// VDD & VRef: connected to pin on STM32 that outputs 3.3V
//
//	// Configure VSS and active low LDAC
//		// VSS & AL LDAC: connected to GND pins to get rid of potential noise from GPIO pins
//
//	// Configure SCK & SDI
//	GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
//	GPIOA->MODER |= (2<<GPIO_MODER_MODE5_Pos);
//	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5_Msk);
//	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_Msk);
//	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
//
//	GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL5);				// select SPI1_SCK function
//	GPIOA->AFR[0]  |= (5 << GPIO_AFRL_AFSEL5_Pos);
//
//	GPIOA->MODER &= ~(GPIO_MODER_MODE7_Msk);
//	GPIOA->MODER |= (2<<GPIO_MODER_MODE7_Pos);
//	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT7_Msk);
//	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7_Msk);
//	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);
//
//	GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);				// select SPI1_MOSI
//	GPIOA->AFR[0]  |= (5<<GPIO_AFRL_AFSEL7_Pos);
//
//
//	// initialize the SPI peripheral to communicate with the DAC
//	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE_Msk);			// 2-line unidirectional data mode bit set to 0
//	SPI1->CR1 &= ~(SPI_CR1_RXONLY_Msk);				// Full - duplex (Transmit and Receive) set bit to 0
//	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST_Msk);			// Frame format data transmitted/received w/ MSB first
//	SPI1->CR1 |= (1<<SPI_CR1_MSTR_Pos);				// Master Configuration
//	SPI1->CR1 |= (1<<SPI_CR1_CPOL_Pos);				// Clock polarity to 0 when idle
//	SPI1->CR1 |= (1<<SPI_CR1_CPHA_Pos);				// Clock phase to first clock transition being the first data capture edge
//
//
//	SPI1->CR2 |= (1<<SPI_CR2_SSOE_Pos);				// SS output enabled in master mode when SPI interface is enabled
//
//	SPI1->CR1 |= (1<<SPI_CR1_SPE_Pos);				// SPI enable
//}
//
//void DAC_write(uint16_t in_val)
//{
//	//  write a 12-bit value to the DAC
//	in_val |= 3<<12;													// add 4 bits of settings to 12-bit val => 16-bit val where MSB is 0011...
//	in_val = ((in_val << 8) & 0xFF00) | ((in_val >> 8) & 0x00FF);		// swap MSB 8 bits with LSB 8 bits to setup in order the DAC will read
//
//	GPIOA->ODR &= ~(GPIO_ODR_OD0_Msk);									// Chip select is set to low since its active low
//	SPI1->DR = (in_val);												// SPI transmits 16-bit value to DAC
//	while (SPI1->SR & (SPI_SR_BSY));									// waits until data is done transmitting when BUSY
//	GPIOA->BSRR |= (1<<GPIO_BSRR_BS0_Pos);								// sets chip select back to high while not doing anything
//}
//
//uint16_t DAC_volt_conv(uint16_t in_val)									// convert a voltage value into a 12-bit value to control the DAC
//{
//	uint16_t v_max = 3300;												// max voltage 3.3 V also the default highest voltage
//
//	uint16_t digital_val = 0;
//	digital_val = (in_val * 4095) / v_max;								// converts value (voltage) inputed into a digital value
//
//	if (digital_val > 4095){
//		digital_val = 4095;
//	}
//	return digital_val;
//
//}
//
//int32_t keypad_read(){
//	int32_t row = read_rows();
//	if(row >= 0){
//		GPIOB->ODR &= 0;
//		GPIOB->BSRR |= (1 << GPIO_ODR_OD4_Pos);							// set first column HIGH
//		for (int col = 0; col < 4 ; col++){
//			for (int delay = 0; delay < 5; delay++);					// delay
//			int row_indx = read_rows();
//			if (row_indx >= 0){
//				return find_key(row_indx, col);
//			}
//			else {
//				GPIOB->ODR <<= 1;
//			}
//		}
//		return -1;
//
//	}
//	else {
//		return -1;
//	}
//}
//
//int32_t read_rows(void){	// return row number
//	if (GPIOB->IDR & (1 << GPIO_IDR_ID0_Pos)){
//		return 0;
//	}
//	if (GPIOB->IDR & (1 << GPIO_IDR_ID1_Pos)){
//		return 1;
//	}
//	if (GPIOB->IDR & (1 << GPIO_IDR_ID2_Pos)){
//		return 2;
//	}
//	if (GPIOB->IDR & (1 << GPIO_IDR_ID3_Pos)){
//		return 3;
//	}
//	else {
//		return -1;
//	}
//}
//
//int32_t find_key(int32_t row_indx, int32_t col){
//	const int32_t key[4][4] = {
//		{1, 2, 3, 10},
//		{4, 5, 6, 11},
//		{7, 8, 9, 12},
//		{15, 0, 14, 13}
//	};
//
//	int32_t key_pressed = key[row_indx][col];
//	return key_pressed;
//}
//
//void TIM2_IRQHandler(void) {
//   if (TIM2->SR & TIM_SR_UIF)
//   {
//	   GPIOA->ODR = (GPIO_ODR_OD1);			 // sets high, rising edge
//       TIM2->SR &= ~(TIM_SR_UIF); 		 // Clear the CC1 interrupt flag (CC1IF)
//       DAC_write(0xABC);
//       GPIOA->ODR = ~(GPIO_ODR_OD1);		 // sets low, falling edge
//   }
//}
//
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
