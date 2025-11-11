/***************************************************************************
 *
 * Author: Elizabeth Acevedo
 *
 * Date: October 24, 2024
 *
 * Program: Simulated Function Generator
 *
 * Description: This project integrates an STM32L476 MCU, a 4x4 keypad,
 *  and an MCP4921 DAC to create a function generator that produces various
 *  waveforms (sine, triangle, sawtooth, and square) with adjustable frequency
 *  (100 Hz to 500 Hz) and duty cycle (10% to 90%). The MCU processes keypad
 *  inputs to control waveform type, frequency, and duty cycle, while the
 *  MCP4921 DAC converts digital values into analog voltages (0 to 3V) for
 *  measurement on an oscilloscope. LEDs display the pressed keys in binary,
 *  providing feedback on keypad functionality.
 *
 **************************************************************************/
/* INCLUDES */
#include "main.h"
#include "math.h"
#include <stdint.h>

/* DEFINES */
#define KEYPAD_PORT GPIOB
#define LED_PORT GPIOC
#define DAC_PORT GPIOA
#define array_size 1680 		// Calculated array size

/* FUNCTION PROTOTYPES */
void SystemClock_Config(void);
void LEDs_init(void);
void TIM2_init(void);
void DAC_init(void);
void DAC_write(uint16_t in_val);
uint16_t DAC_volt_conv(uint16_t in_val);
int32_t keypad_read(void);
int32_t read_rows(void);
int32_t find_key(int32_t row_indx, int32_t col);

/* GLOBAL VARIABLES */
int32_t sin_wav[array_size];
int32_t tri_wav[array_size];
int32_t sawt_wav[array_size];
int32_t sq_wav = 0;
int32_t *wave = sin_wav;
int32_t duty = array_size / 10;
int32_t duty_cycle = 5;
int32_t frequency = 1;
int32_t sample_indx = 0;


int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();


  // Pre-calculated arrays for each waveform (in mV)
  for (int i = 0; i < array_size; i++){
	  // sin wave calculation
	  sin_wav[i] = round(1500 * sin((2 * M_PI * i) / array_size) + 1500); // scale * (2pi - full cycle) * offset

	  // triangle wave calculation
	  if (i < array_size/2){
		  // counting-up
		  tri_wav[i] = round(3000 * (i) / (array_size/2));
	  }
	  else {
		  // counting-down
		  tri_wav[i] = round(3000 * (array_size - i) / (array_size/2));
	  }

	  // sawtooth wave calculation
	  sawt_wav[i] = round(3000 * (i) / (array_size));

  }


  RCC->AHB2ENR   |= RCC_AHB2ENR_GPIOBEN;				// Enable GPIOB Clock for ports on board rows/cols
  RCC->AHB2ENR	 |= RCC_AHB2ENR_GPIOCEN;				// Enable GPIOC Clock for LEDS


  TIM2_init();
  DAC_init();
  LEDs_init();

  /* Configure Rows */
  KEYPAD_PORT->MODER  &= ~(GPIO_MODER_MODE0_Msk | GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);	// prepare to read row 0
  KEYPAD_PORT->PUPDR  &= ~(GPIO_PUPDR_PUPD0_Msk | GPIO_PUPDR_PUPD1_Msk | GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk);		//
  KEYPAD_PORT->PUPDR  |= ((2 << GPIO_PUPDR_PUPD0_Pos) | (2 << GPIO_PUPDR_PUPD1_Pos) | (2 << GPIO_PUPDR_PUPD2_Pos) | (2 << GPIO_PUPDR_PUPD3_Pos));	// pull-down


  /* Configure Columns */
  KEYPAD_PORT->MODER	&= ~(GPIO_MODER_MODE4_Msk | GPIO_MODER_MODE5_Msk | GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk);	// output mode
  KEYPAD_PORT->MODER 	|= ((1 << GPIO_MODER_MODE4_Pos) | (1 << GPIO_MODER_MODE5_Pos) | (1 << GPIO_MODER_MODE6_Pos) | (1 << GPIO_MODER_MODE7_Pos));	// mode to 01
  KEYPAD_PORT->OTYPER	&= ~(GPIO_OTYPER_OT4_Msk | GPIO_OTYPER_OT5_Msk | GPIO_OTYPER_OT6_Msk | GPIO_OTYPER_OT7_Msk);	// push-pull output
  KEYPAD_PORT->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED4_Msk | GPIO_OSPEEDR_OSPEED5_Msk | GPIO_OSPEEDR_OSPEED6_Msk | GPIO_OSPEEDR_OSPEED7_Msk);	//low speed
  KEYPAD_PORT->PUPDR 	&= ~(GPIO_PUPDR_PUPD4_Msk | GPIO_PUPDR_PUPD5_Msk | GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk);	//no resistor


  /* Set columns high */
  KEYPAD_PORT->BSRR |= ((1<<GPIO_ODR_OD4_Pos) | (1<<GPIO_ODR_OD5_Pos) | (1<<GPIO_ODR_OD6_Pos) | (1<<GPIO_ODR_OD7_Pos));


  LED_PORT->ODR &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);	// Clear LEDS


  while (1)
  {

	int32_t disp_key = keypad_read();			// disp_key is holding my key number 0-15

	if (disp_key != -1) {						// Key was pressed
		LED_PORT->ODR &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);					// Clear output register for LEDS

		LED_PORT->BSRR |= (disp_key);			// Displays key on LEDS


		  switch (disp_key)
		  {
		  	  case 1:
		  		  // waveform frequency to 100Hz
		  		  frequency = 1;
		  		  break;
		  	  case 2:
		  		// waveform frequency to 200Hz
		  		frequency = 2;
		  		break;
		  	  case 3:
		  		// waveform frequency to 300Hz
		  		frequency = 3;
		  		break;
		  	  case 4:
		  		// waveform frequency to 400Hz
		  		frequency = 4;
		  		break;
		  	  case 5:
		  		// waveform frequency to 500Hz
		  		frequency = 5;
		  		break;
		  	  case 6:
		  		// sin wave
		  		wave = sin_wav;
		  		sq_wav = 0;
		  		break;
		  	  case 7:
		  		// triangle wave
		  		wave = tri_wav;
		  		sq_wav = 0;
		  		break;
		  	  case 8:
		  		// sawtooth wave
		  		wave = sawt_wav;
		  		sq_wav = 0;
		  		break;
		  	  case 9:
		  		// square wave
		  		sq_wav = 1;
		  		break;
		  	  case 14:
		  		// increases duty cycle by 10% up to a maximum of 90%
		  		if (duty_cycle < 9) {
		  			duty_cycle += 1;
		  		}
		  		break;
		  	  case 0:
		  		// reset duty cycle to 50%
		  		  duty_cycle = 5;
		  		break;
		  	  case 15:
		  		// decreases duty cycle by 10% down to min of 10%
		  		if (duty_cycle > 1) {
		  			duty_cycle -= 1;
		  		}
		  		break;

		  	  default:
		  		break;
		  }

		  for (int i = 0; i < 255000; i++);		// Delay so it can take a key at a time

	}
	/* Set columns high */
	KEYPAD_PORT->BSRR |= ((1<<GPIO_ODR_OD4_Pos) | (1<<GPIO_ODR_OD5_Pos) | (1<<GPIO_ODR_OD6_Pos) | (1<<GPIO_ODR_OD7_Pos));
  }

}

void LEDs_init(void)
{ // LED Configuration
	/* Pins 0-3 (four pins) are cleared and set to general purpose output mode */
	LED_PORT->MODER	&= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	LED_PORT->MODER   |=  (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0);

	/* Setting to output push-pull (reset state)*/
	LED_PORT->OTYPER  &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);

	/* Set to no pull-up, pull-down*/
	LED_PORT->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);

	/* Set to very high speed*/
	LED_PORT->OSPEEDR |=  ((3 << GPIO_OSPEEDR_OSPEED0_Pos)	|	(3 << GPIO_OSPEEDR_OSPEED1_Pos) | (3 << GPIO_OSPEEDR_OSPEED2_Pos) | (3 << GPIO_OSPEEDR_OSPEED3_Pos));
}

void TIM2_init(void)
{
	// TIM2 Configuration
	RCC->APB1ENR1  |= (RCC_APB1ENR1_TIM2EN);		// TIM2 clock enable

	TIM2->ARR = 238;								// precaclulated ARR value (236)
	// TIM2 Control Register Configuration
	TIM2->CR1 = 0;									// Default timer settings
	TIM2->CR1 &= ~(TIM_CR1_DIR); 						// Sets counter direction up, (0: counts up)
	TIM2->CR1 |= (TIM_CR1_URS); 						// Set to 1 --> Only overflow/underflow generates an update interrupt
	TIM2->CR1 &= ~(TIM_CR1_UDIS); 					// Enable update events (UEV), bit cleared to 0
	// Enabling
	TIM2->DIER |= TIM_DIER_UIE; 						// Enable update interrupt (bit 0 set to 1)
	NVIC->ISER[0] |= (1<<TIM2_IRQn);					// Enable TIM2 interrupt in the NVIC
	TIM2->CR1 |= TIM_CR1_CEN; 						// Counter enabled, timer starts, (bit 0 is set to 1)

}

void DAC_init(void)
{
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN); 			// enables GPIOA clock used for DAC
	RCC->APB2ENR   |=  (RCC_APB2ENR_SPI1EN);			// SPI clock enable

	// Configure chip select pin
	GPIOA->MODER &= ~(GPIO_MODER_MODE0_Msk);
	GPIOA->MODER |= (1<<GPIO_MODER_MODE0_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0_Msk);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0_Msk);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD0_Msk);

	GPIOA->BSRR |= (1<<GPIO_BSRR_BS0_Pos);				// CS high when not being used

	// Configure VDD and VRef
		// VDD & VRef: connected to pin on STM32 that outputs 3.3V

	// Configure VSS and active low LDAC
		// VSS & AL LDAC: connected to GND pins to get rid of potential noise from GPIO pins

	// Configure SCK & SDI
	GPIOA->MODER &= ~(GPIO_MODER_MODE5_Msk);
	GPIOA->MODER |= (2<<GPIO_MODER_MODE5_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5_Msk);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED5_Msk);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5_Msk);
	GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL5);				// Select SPI1_SCK function
	GPIOA->AFR[0]  |= (5 << GPIO_AFRL_AFSEL5_Pos);

	GPIOA->MODER &= ~(GPIO_MODER_MODE7_Msk);
	GPIOA->MODER |= (2<<GPIO_MODER_MODE7_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT7_Msk);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7_Msk);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD7_Msk);
	GPIOA->AFR[0]  &= ~(GPIO_AFRL_AFSEL7);				// Select SPI1_MOSI
	GPIOA->AFR[0]  |= (5<<GPIO_AFRL_AFSEL7_Pos);


	// initialize the SPI peripheral to communicate with the DAC
	SPI1->CR1 &= ~(SPI_CR1_BIDIMODE_Msk);			// 2-line unidirectional data mode bit set to 0
	SPI1->CR1 &= ~(SPI_CR1_RXONLY_Msk);				// Full - duplex (Transmit and Receive) set bit to 0
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST_Msk);			// Frame format data transmitted/received w/ MSB first
	SPI1->CR1 |= (1<<SPI_CR1_MSTR_Pos);				// Master Configuration
	SPI1->CR1 |= (1<<SPI_CR1_CPOL_Pos);				// Clock polarity to 0 when idle
	SPI1->CR1 |= (1<<SPI_CR1_CPHA_Pos);				// Clock phase to first clock transition being the first data capture edge


	SPI1->CR2 |= (1<<SPI_CR2_SSOE_Pos);				// SS output enabled in master mode when SPI interface is enabled

	SPI1->CR1 |= (1<<SPI_CR1_SPE_Pos);				// SPI enable
}

void TIM2_IRQHandler(void) {
   if (TIM2->SR & TIM_SR_UIF)						// Checks if UIF flag is set, timer reached its completing period
   {
       TIM2->SR &= ~(TIM_SR_UIF);					// Clears the update interrupt flag (UIF)
       if (sq_wav == 1) {							// Square wave flag is high, no longer low
    	   if (sample_indx < (duty * duty_cycle)) {	// Checks if array index is less than duty cycle determining a high or low
    		   DAC_write(DAC_volt_conv(3000));		// Square wave high
    	   }
    	   else {
    		   DAC_write(DAC_volt_conv(0));			// Square wave low
    	   }
       }
       else { // Outputs other waveforms from pre-calculations: sin, triangle or sawtooth
    	   DAC_write(DAC_volt_conv(wave[sample_indx]));
       }

       /* FREQUENCY */
       if (sample_indx < array_size)				// checks if array index is less than array size: Iterates through array (sin, tri, or sawtooth)
    	   sample_indx += frequency;				// Default freq is at 100Hz, incrementing array index by 2, 3, 4, or 5 changes freq.
       else
    	   sample_indx = 0;							// resets index to the beginning of the array
   }
}

void DAC_write(uint16_t in_val)
{
	//  write a 12-bit value to the DAC
	in_val |= 3<<12;													// add 4 bits of settings to 12-bit val => 16-bit val where MSB is 0011...
	in_val = ((in_val << 8) & 0xFF00) | ((in_val >> 8) & 0x00FF);		// swap MSB 8 bits with LSB 8 bits to setup in order the DAC will read

	GPIOA->ODR &= ~(GPIO_ODR_OD0_Msk);									// Chip select is set to low since its active low
	SPI1->DR = (in_val);												// SPI transmits 16-bit value to DAC
	while (SPI1->SR & (SPI_SR_BSY));									// waits until data is done transmitting when BUSY
	GPIOA->BSRR |= (1<<GPIO_BSRR_BS0_Pos);								// sets chip select back to high while not doing anything
}

uint16_t DAC_volt_conv(uint16_t in_val)									// convert a voltage value into a 12-bit value to control the DAC
{
	uint16_t v_max = 3300;												// max voltage 3.3 V also the default highest voltage

	uint16_t digital_val = 0;
	digital_val = (in_val * 4095) / v_max;								// converts value (voltage) inputed into a digital value

	if (digital_val > 4095){											// scales voltage so it does not exceed 3.3V or 3300 mV
		digital_val = 4095;
	}
	return digital_val;

}

int32_t keypad_read() {
	int32_t row = read_rows();
	if(row >= 0){
		GPIOB->ODR &= 0;
		GPIOB->BSRR |= (1 << GPIO_ODR_OD4_Pos);							// Set first column HIGH
		for (int col = 0; col < 4 ; col++){
			for (int delay = 0; delay < 5; delay++);					// Delay
			int row_indx = read_rows();
			if (row_indx >= 0){
				return find_key(row_indx, col);							// returns key pressed at specific row/col
			}
			else {
				GPIOB->ODR <<= 1;										// moves to next column
			}
		}
		return -1;

	}
	else {
		return -1;
	}
}

int32_t read_rows(void){	// return row number
	// reads rows 0 - 3, otherwise return -1
	if (GPIOB->IDR & (1 << GPIO_IDR_ID0_Pos)){
		return 0;
	}
	if (GPIOB->IDR & (1 << GPIO_IDR_ID1_Pos)){
		return 1;
	}
	if (GPIOB->IDR & (1 << GPIO_IDR_ID2_Pos)){
		return 2;
	}
	if (GPIOB->IDR & (1 << GPIO_IDR_ID3_Pos)){
		return 3;
	}
	else {
		return -1;
	}
}

int32_t find_key(int32_t row_indx, int32_t col){
	const int32_t key[4][4] = {						// keypad array of keys
		{1, 2, 3, 10},
		{4, 5, 6, 11},
		{7, 8, 9, 12},
		{15, 0, 14, 13}
	};

	int32_t key_pressed = key[row_indx][col];		// indexing array
	return key_pressed;								// holds key at specific row/col
}



void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
