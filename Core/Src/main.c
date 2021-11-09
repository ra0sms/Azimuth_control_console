/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hd44780.h"



uint8_t flag_key1_press = 1;
uint32_t time_key1_press = 0;
uint8_t flag_key2_press = 1;
uint32_t time_key2_press = 0;
uint8_t flag_key3_press = 1;
uint32_t time_key3_press = 0;
uint16_t adc_value = 0;
uint8_t flag_adc = 0;
uint8_t flag_eeprom = 0;


void ShowStartAzimuth();
void ReadCWButton();
void ReadCCWButton();
void ReadStartButton();
void WriteToEEPROM (uint32_t address, uint32_t value);
uint32_t ReadFromEEPROM (uint32_t address);
void SaveSettings();
void USART2_Send (char chr);
void USART2_Send_String (char* str);
void ConvertGradusToChar (uint32_t grad);
void CheckUSART();
void RotateFromUSART(uint32_t grad);
uint32_t ConvertCharToGradus ();
void CheckADC();
void ShowWarning();
void SetDefault();




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

void SetDefault(){
	while (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
		lcdClrScr();
		lcdGoto(LCD_1st_LINE, 3);
		lcdPuts("GO TO ZERO");
		lcdGoto(LCD_2nd_LINE, 16);
		LL_mDelay(300);
	}
	gradus = man_azimuth = 0;
	WriteToEEPROM(EEPROM_ADDRESS_START, gradus);
    WriteToEEPROM(EEPROM_ADDRESS_START+sizeof(gradus), man_azimuth);
}

void ShowWarning() {
	lcdClrScr();
	lcdGoto(LCD_1st_LINE, 1);
	lcdPuts("TURN ON 12VDC!!");
	lcdGoto(LCD_2nd_LINE, 16);
}

void CheckADC()
{
	LL_ADC_Enable(ADC1);
    LL_ADC_REG_StartConversion(ADC1);
    LL_mDelay(1);
    adc_value = LL_ADC_REG_ReadConversionData12(ADC1);
    if ((adc_value < 3500)&&(flag_adc == 0)) {
    	if (flag_eeprom==1){
    		SaveSettings();
    		flag_eeprom=0;
    	}
    	flag_adc = 1;
    	ShowWarning();
    }
    if ((adc_value > 3500)&&(flag_adc == 1)) {
    	ShowStartAzimuth();
    	flag_adc = 0;
    }
}

void RotateFromUSART(uint32_t grad)
 {
	man_azimuth = grad;
	if (man_azimuth >= 180) dir_azimuth = man_azimuth - 360; else dir_azimuth = man_azimuth;
    if (gradus >= 180) dir_gradus = gradus - 360; else dir_gradus = gradus;
    lcdGoto(LCD_2nd_LINE,13);
    lcdPuts("   ");
    lcdGoto(LCD_2nd_LINE,13);
    lcdItos(man_azimuth);
    lcdGoto(LCD_2nd_LINE,16);
	LL_TIM_ClearFlag_UPDATE(TIM2);
	LL_TIM_EnableCounter(TIM2);
	LL_TIM_EnableIT_UPDATE(TIM2);
	if (dir_azimuth > dir_gradus) {
		time_on_cw = HAL_GetTick();
		while (dir_gradus < dir_azimuth) {
			lcdGoto(LCD_1st_LINE, 13);
			lcdPuts("   ");
			lcdGoto(LCD_1st_LINE, 13);
			lcdItos(gradus);
			lcdGoto(LCD_1st_LINE, 16);
			LL_GPIO_ResetOutputPin(OE_GPIO_Port, OE_Pin);
			LL_GPIO_SetOutputPin(CW_GPIO_Port, CW_Pin);
			LL_mDelay(50);
			if ((HAL_GetTick() - time_on_cw) > 10000)
				break;
			if (flag_stop==1) {flag_stop =0; break;}
		}
		LL_GPIO_SetOutputPin(OE_GPIO_Port, OE_Pin);
		LL_GPIO_ResetOutputPin(CW_GPIO_Port, CW_Pin);
		LL_TIM_DisableCounter(TIM2);
		LL_TIM_DisableIT_UPDATE(TIM2);

	} else {
		time_on_ccw = HAL_GetTick();
		while (dir_gradus > dir_azimuth) {
			lcdGoto(LCD_1st_LINE, 13);
			lcdPuts("   ");
			lcdGoto(LCD_1st_LINE, 13);
			lcdItos(gradus);
			lcdGoto(LCD_1st_LINE, 16);
			LL_GPIO_ResetOutputPin(OE_GPIO_Port, OE_Pin);
			LL_GPIO_SetOutputPin(CCW_GPIO_Port, CCW_Pin);
			LL_mDelay(50);
			if ((HAL_GetTick() - time_on_ccw) > 10000)
				break;
			if (flag_stop==1) {flag_stop =0; break;}
		}
		LL_GPIO_SetOutputPin(OE_GPIO_Port, OE_Pin);
		LL_GPIO_ResetOutputPin(CCW_GPIO_Port, CCW_Pin);
		LL_TIM_DisableCounter(TIM2);
		LL_TIM_DisableIT_UPDATE(TIM2);
	}
	lcdGoto(LCD_1st_LINE, 13);
	lcdPuts("   ");
	lcdGoto(LCD_1st_LINE, 13);
	lcdItos(gradus);
	lcdGoto(LCD_1st_LINE, 16);
	ConvertGradusToChar(gradus);

}

uint32_t ConvertCharToGradus() {
	uint8_t hund = 0;
	uint8_t dec = 0;
	uint8_t one = 0;
	uint8_t ostatok = 0;
	uint32_t rezult = 0;
	hund = str_rx[1] - 48;
	dec = str_rx[2] - 48;
	one = str_rx[3] - 48;
	rezult = hund * 100 + dec * 10 + one;
	ostatok = rezult % 3;
	str_rx[0] = 0;
	str_rx[1] = 0;
	str_rx[2] = 0;
	str_rx[3] = 0;
	str_rx[4] = 0;
	str_rx[5] = 0;
	if (ostatok == 0)
		return rezult;
	else
		return (rezult - ostatok);

}

void CheckUSART()
{
	if (flag_status == 1)
		{
		flag_status=0;
		ConvertGradusToChar(gradus);
		str_rx[0]=0;
		str_rx[1]=0;
		str_rx[2]=0;
		str_rx[3]=0;
		str_rx[4]=0;
		str_rx[5]=0;
		USART2_Send_String(str_tx);
		}
	if (flag_move == 1)
	{
		flag_move = 0;
		RotateFromUSART(ConvertCharToGradus());
	}

}

void ConvertGradusToChar (uint32_t grad)
{
	uint8_t hund=0;
	uint8_t dec=0;
	uint8_t one=0;
	if (grad >= 100)
		{
		hund = grad / 100;
		dec = (grad % 100)/10;
		one = dec % 10;
		}
	if (grad < 10)
	{
		hund = 0;
		dec = 0;
		one = grad;
	}
	if ((grad>9)&&(grad<100))
	{
		hund = 0;
		dec = grad/10;
		one = grad % 10;
	}
	sprintf (str_tx, "+0%d%d%d\r\n",hund,dec,one);
}

void USART2_Send (char chr){
	while (!(USART2->ISR & USART_ISR_TC));
	USART2->TDR = chr;
}

void USART2_Send_String (char* str){
	uint8_t i = 0;
	while(str[i])
	USART2_Send (str[i++]);
}

void SaveSettings()
{
	WriteToEEPROM(EEPROM_ADDRESS_START, gradus);
	WriteToEEPROM(EEPROM_ADDRESS_START+sizeof(gradus), man_azimuth);
}

void ShowStartAzimuth ()
{
	lcdClrScr();
	lcdGoto(LCD_1st_LINE,1);
	lcdPuts("CUR AZIMUTH");
	lcdGoto(LCD_1st_LINE,13);
	lcdItos(ReadFromEEPROM(EEPROM_ADDRESS_START));
	lcdGoto(LCD_2nd_LINE,1);
	lcdPuts("ANT AZIMUTH");
	lcdGoto(LCD_2nd_LINE,13);
	lcdItos(ReadFromEEPROM(EEPROM_ADDRESS_START+sizeof(gradus)));
	lcdGoto(LCD_2nd_LINE,16);
}

void ReadCWButton() {
	if (HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin) == GPIO_PIN_RESET
			&& flag_key1_press) {
		flag_key1_press = 0;
		if (man_azimuth==359) man_azimuth=0;
		man_azimuth=man_azimuth+1;
		isPushCW = 1;
		LL_GPIO_ResetOutputPin(CW_GPIO_Port, CW_Pin);
		lcdGoto(LCD_2nd_LINE,13);
		lcdPuts("   ");
		lcdGoto(LCD_2nd_LINE,13);
	    lcdItos(man_azimuth);
	    lcdGoto(LCD_2nd_LINE,16);
		time_key1_press = HAL_GetTick();
	}

	if (!flag_key1_press && (HAL_GetTick() - time_key1_press) > 200) {
		flag_key1_press = 1;
	}
}
void ReadCCWButton() {
	if (HAL_GPIO_ReadPin(BTN_DWN_GPIO_Port, BTN_DWN_Pin) == GPIO_PIN_RESET
			&& flag_key2_press) {
		flag_key2_press = 0;
		if (man_azimuth==0) man_azimuth=359;
		man_azimuth=man_azimuth-1;
		isPushCW = 0;
		LL_GPIO_SetOutputPin(OE_GPIO_Port, OE_Pin);
		lcdGoto(LCD_2nd_LINE,13);
		lcdPuts("   ");
		lcdGoto(LCD_2nd_LINE,13);
	    lcdItos(man_azimuth);
	    lcdGoto(LCD_2nd_LINE,16);
		time_key2_press = HAL_GetTick();
	}

	if (!flag_key2_press && (HAL_GetTick() - time_key2_press) > 200) {
		flag_key2_press = 1;
	}
}
void ReadStartButton() {
	if (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET
			&& flag_key3_press) {
		flag_key3_press = 0;
		//SaveSettings();
		LL_TIM_ClearFlag_UPDATE(TIM2);
		LL_TIM_EnableCounter(TIM2);
		LL_TIM_EnableIT_UPDATE(TIM2);
		if (dir_azimuth > dir_gradus) {
			time_on_cw = HAL_GetTick();
			while (dir_gradus < dir_azimuth) {
				lcdGoto(LCD_1st_LINE, 13);
				lcdPuts("   ");
				lcdGoto(LCD_1st_LINE, 13);
				lcdItos(gradus);
				lcdGoto(LCD_1st_LINE, 16);
				LL_GPIO_ResetOutputPin(OE_GPIO_Port, OE_Pin);
				LL_GPIO_SetOutputPin(CW_GPIO_Port, CW_Pin);
				LL_mDelay(50);
				if ((HAL_GetTick() - time_on_cw) > 10000) break;
			}
			LL_GPIO_SetOutputPin(OE_GPIO_Port, OE_Pin);
			LL_GPIO_ResetOutputPin(CW_GPIO_Port, CW_Pin);
			LL_TIM_DisableCounter(TIM2);
			LL_TIM_DisableIT_UPDATE(TIM2);

		} else {
			time_on_ccw = HAL_GetTick();
			while (dir_gradus > dir_azimuth) {
				lcdGoto(LCD_1st_LINE, 13);
				lcdPuts("   ");
				lcdGoto(LCD_1st_LINE, 13);
				lcdItos(gradus);
				lcdGoto(LCD_1st_LINE, 16);
				LL_GPIO_ResetOutputPin(OE_GPIO_Port, OE_Pin);
				LL_GPIO_SetOutputPin(CCW_GPIO_Port, CCW_Pin);
				LL_mDelay(50);
				if ((HAL_GetTick() - time_on_ccw) > 10000) break;
			}
			LL_GPIO_SetOutputPin(OE_GPIO_Port, OE_Pin);
			LL_GPIO_ResetOutputPin(CCW_GPIO_Port, CCW_Pin);
			LL_TIM_DisableCounter(TIM2);
			LL_TIM_DisableIT_UPDATE(TIM2);
		}
		lcdGoto(LCD_1st_LINE, 13);
		lcdPuts("   ");
	    lcdGoto(LCD_1st_LINE, 13);
		lcdItos(gradus);
		lcdGoto(LCD_1st_LINE, 16);
		ConvertGradusToChar(gradus);
		time_key3_press = HAL_GetTick();
	}

	if (!flag_key3_press && (HAL_GetTick() - time_key3_press) > 200) {
		flag_key3_press = 1;
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM21_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void WriteToEEPROM (uint32_t address, uint32_t value)
{
  HAL_StatusTypeDef flash_ok = HAL_ERROR;
  while (flash_ok != HAL_OK)
  {
    flash_ok = HAL_FLASHEx_DATAEEPROM_Unlock();
  }
  flash_ok = HAL_ERROR;
  while (flash_ok != HAL_OK)
  {
    flash_ok = HAL_FLASHEx_DATAEEPROM_Erase (address);
  }
  flash_ok = HAL_ERROR;
  while (flash_ok != HAL_OK)
  {
    flash_ok = HAL_FLASHEx_DATAEEPROM_Program (FLASH_TYPEPROGRAMDATA_WORD, address, value);
  }
  flash_ok = HAL_ERROR;
  while (flash_ok != HAL_OK)
  {
    flash_ok = HAL_FLASHEx_DATAEEPROM_Lock ();
  }
}

uint32_t ReadFromEEPROM (uint32_t address)
{
  return (*(__IO uint32_t *)address);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM21_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM21);
  LL_TIM_EnableIT_UPDATE(TIM21);
  LL_USART_Enable(USART2);
  LL_USART_EnableIT_RXNE(USART2);
  //LL_mDelay(400);
  //uint32_t SystemFreq = HAL_RCC_GetHCLKFreq();
  lcdInit();
  lcdBackLightOn();
  lcdClrScr();

  imp_count = 0;
  isPushCW = 2;
  dir_azimuth = 0;
  dir_gradus = 0;
  time_on_cw = 0;
  time_on_ccw = 0;
  flag_stop = 0;
  flag_status = 0;
  flag_move=0;
  if (HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin) == GPIO_PIN_RESET){
	  SetDefault();
  }
  gradus = ReadFromEEPROM(EEPROM_ADDRESS_START);
  man_azimuth = ReadFromEEPROM(EEPROM_ADDRESS_START + sizeof(man_azimuth));
  imp_count = gradus;
  ShowStartAzimuth ();
  str_rx[0] = 0;
  str_rx[1] = 0;
  str_rx[2] = 0;
  str_rx[3] = 0;
  str_rx[4] = 0;
  str_rx[5] = 0;
   LL_ADC_StartCalibration(ADC1);
   LL_mDelay(5);
   CheckADC();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		CheckADC();
		if (flag_adc == 0) {
			flag_eeprom=1;
			if (man_azimuth >= 180)
				dir_azimuth = man_azimuth - 360;
			else
				dir_azimuth = man_azimuth;
			if (gradus >= 180)
				dir_gradus = gradus - 360;
			else
				dir_gradus = gradus;
			ReadCWButton();
			ReadCCWButton();
			ReadStartButton();
			CheckUSART();
		}
		LL_mDelay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_InitTypeDef ADC_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**ADC GPIO Configuration
  PA0-CK_IN   ------> ADC_IN0
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* ADC interrupt Init */
  NVIC_SetPriority(ADC1_IRQn, 0);
  NVIC_EnableIRQ(ADC1_IRQn);

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
  /** Common config
  */
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_12CYCLES_5);
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
  LL_ADC_DisableIT_EOC(ADC1);
  LL_ADC_DisableIT_EOS(ADC1);
  ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);

  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**TIM2 GPIO Configuration
  PA4   ------> TIM2_ETR
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, 0);
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 2;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV2;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_ConfigETR(TIM2, LL_TIM_ETR_POLARITY_INVERTED, LL_TIM_ETR_PRESCALER_DIV1, LL_TIM_ETR_FILTER_FDIV32_N6);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_EXT_MODE2);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);

  /* TIM21 interrupt Init */
  NVIC_SetPriority(TIM21_IRQn, 0);
  NVIC_EnableIRQ(TIM21_IRQn);

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  TIM_InitStruct.Prescaler = 31999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM21, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM21);
  LL_TIM_SetClockSource(TIM21, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM21, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM21);
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, 0);
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(OE_GPIO_Port, OE_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CW_GPIO_Port, CW_Pin);

  /**/
  LL_GPIO_ResetOutputPin(CCW_GPIO_Port, CCW_Pin);

  /**/
  GPIO_InitStruct.Pin = BTN_UP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_UP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_DWN_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_DWN_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OE_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OE_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(CW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = CCW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(CCW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_START_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_START_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
