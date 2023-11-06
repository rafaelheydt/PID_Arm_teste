/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nokia5110_LCD.h"
#include <stdio.h>
#include <stdlib.h>


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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t mode = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const int selecionarPino[4] = {S0_Pin, S1_Pin, S2_Pin, S3_Pin};

uint16_t valorSensor[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Valores Analogico Sensores
int sensorDigital[16] = 	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Valores Digitais Sensores
uint16_t corBranco[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Para calibração cor branca
uint16_t corPreto[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Para calibração cor preta
uint16_t mediaPB[16] ={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // Valor média calibração linha e chão
uint16_t peso[16] = {1500, 1400, 1300, 1200, 1100, 1000, 900, 800, 700, 600, 500, 400, 300, 200, 100, 0}; //vetor com os pesos
uint32_t PWMA = 0;
uint32_t PWMB = 0;
uint16_t pos = 0;
int linha = 0; // 0 -> Linha preta // 1-> Linha Branco
char Buffer[20];

/* Variáveis PID --------------------------------------------------------------------*/
int error=0; // Posição- (Maior peso)/2
//constantes PID
uint16_t Kp = 2;
uint16_t Kd=0;
uint16_t Ki=0;

//constantes auxiliares PID
int propo;
int deriv;
uint16_t integral;
uint16_t ultimopropo=0;

//velocidades base
uint16_t Velo1= 500; // Motor Direita
uint16_t Velo2= 800; // Motor Esquerda
uint16_t velomax= 2506;//4560




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BOT1_Pin) // Ações ao apertar o botão 1
	{
		LCD_clrScr();
		mode += 1;
		if(mode == 3)
		{
			mode = 2;
		}
	}
	if(GPIO_Pin == BOT2_Pin) // Ações ao apertar botão 2
	{

	}
}

void selecionarPinoMux(int pino)
{
	for(int i =0;i<4;i++)
	{
		if(i == 0)
		{
			if(pino & (1<<i))
			{
				HAL_GPIO_WritePin(GPIOC, selecionarPino[i], GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOC, selecionarPino[i], GPIO_PIN_RESET);
			}
		}
		else
		{
			if(pino & (1<<i))
			{
				HAL_GPIO_WritePin(GPIOA, selecionarPino[i], GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOA, selecionarPino[i], GPIO_PIN_RESET);
			}
		}

	}
}

void calibrarBranco() // Função de Medir a cor Branca
{
	for(int i = 0; i<6; i++)
	{
			selecionarPinoMux(i);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			corBranco[5-i] = HAL_ADC_GetValue(&hadc1);
	}
	for(int j = 10; j<16; j++ )
	{
		selecionarPinoMux(j);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		corBranco[25-j] = HAL_ADC_GetValue(&hadc1);
	}
	for(int k = 6; k<10; k++)
	{
		selecionarPinoMux(k);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		corBranco[k] = HAL_ADC_GetValue(&hadc1);
	}
}

void calibrarPreto() // Função de medir a cor Preta
{
	for(int i = 0; i<6; i++)
	{
		selecionarPinoMux(i);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		corPreto[5-i] = HAL_ADC_GetValue(&hadc1);
	}
	for(int j = 10; j<16; j++ )
	{
		selecionarPinoMux(j);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		corPreto[25-j] = HAL_ADC_GetValue(&hadc1);
	}
	for(int k = 6; k<10; k++)
	{
		selecionarPinoMux(k);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		corPreto[k] = HAL_ADC_GetValue(&hadc1);
	}
}

void calcularMediaSensores() // Calculo Média das cores para calibração dos sensores
{
	for(int i = 0;i<16;i++)
	{
		mediaPB[i] = (corPreto[i]+ corBranco[i])/2;
	}
}

void aplicarCalibracao() // Determina os valores digitais dos sensores apos calibração
{
	for(int i = 0; i<6; i++)
	{
		selecionarPinoMux(i);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		valorSensor[5-i] = HAL_ADC_GetValue(&hadc1);
		if(linha == 0)
		{
			if(valorSensor[5-i] <= mediaPB[5-i])
			{
				sensorDigital[5-i] = 0;
			}
			else
			{
				sensorDigital[5-i] = 1;
			}
		}
		if(linha == 1)
		{
			if(valorSensor[5-i] <= mediaPB[5-i])
			{
				sensorDigital[5-i] = 1;
			}
			else
			{
				sensorDigital[5-i] = 0;
			}
		}
	}
	for(int j = 10; j<16; j++ )
	{
		selecionarPinoMux(j);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		valorSensor[25-j] = HAL_ADC_GetValue(&hadc1);
		if(linha == 0)
		{
			if(valorSensor[25-j] <= mediaPB[25-j])
			{
				sensorDigital[25-j] = 0;
			}
			else
			{
				sensorDigital[25-j] = 1;
			}
		}
		if(linha == 1)
		{
			if(valorSensor[25-j] <= mediaPB[25-j])
			{
				sensorDigital[25-j] = 0;
			}
			else
			{
				sensorDigital[25-j] = 1;
			}
		}
	}
	for(int k = 6; k<10; k++)
	{
		selecionarPinoMux(k);
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		valorSensor[k] = HAL_ADC_GetValue(&hadc1);
		if(linha == 0)
		{
			if(valorSensor[k] <= mediaPB[k])
			{
				sensorDigital[k] = 0;
			}
			else
			{
				sensorDigital[k] = 1;
			}
		}
		if(linha == 1)
		{
			if(valorSensor[k] <= mediaPB[k])
			{
				sensorDigital[k] = 1;
			}
			else
			{
				sensorDigital[k] = 0;
			}
		}
	}
}

void leituraLinha() // Retorna Valor da média ponderada para utilizar no PID
{
	    /*Definindo e resetando variaveis*/

	    int num = 0; // numerador
	    int den = 0; // denominador
	    pos = 0;


	    for(int i = 0; i < 16; i++) {
	        // soma ponderada
	        num += sensorDigital[i] * peso[i];
	        den += sensorDigital[i];
	    }

	    if (den != 0) {
	        pos = (num / den);

	       // LCD_print("pos", 1, 0);
	        //sprintf(pos, "%d", pos);
	       // LCD_print(pos,1, 6);
	    } else {
	    	LCD_print("erro0", 3, 0);
	    }

}

void PID(){
	/* Essa função atualiza os valores das variáveis PWMA e PWMB, as variáveis veloA e veloB forma a velocidade base

	 */
                         error = (pos -750);
                         propo= error;                         //função proporcional
                         deriv=propo-ultimopropo;             //função derivativo
                         integral=propo+deriv;                //função integral
                         ultimopropo=propo;

                         PWMA =(Velo1 +((Kp*propo)+(deriv*Kd)+(integral*Ki)));
                         PWMB =(Velo2 -((Kp*propo)+(deriv*Kd)+(integral*Ki)));

                         if(PWMA>velomax){
                        	 PWMA=velomax;
                         }
                         if(PWMB>velomax){
                        	 PWMB=velomax;
                         }

}
void setPWM()
{

	TIM2->CCR2= PWMA;
	TIM1->CCR1 = PWMB;

}

void ligarMotorA() // Motor A-> Esquerda
{
	HAL_GPIO_WritePin(STBY_GPIO_Port,STBY_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AI1_GPIO_Port,AI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AI2_GPIO_Port,AI2_Pin, GPIO_PIN_RESET);
}

void ligarMotorB() // Motor B-> Direita
{

	HAL_GPIO_WritePin(BI1_GPIO_Port,BI1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BI2_GPIO_Port,BI2_Pin, GPIO_PIN_RESET);
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
  LCD_setRST(RST_GPIO_Port, RST_Pin);
  LCD_setCE(CE_GPIO_Port, CE_Pin);
  LCD_setDC(DC_GPIO_Port, DC_Pin);
  LCD_setDIN(DIN_GPIO_Port, DIN_Pin);
  LCD_setCLK(CLK_GPIO_Port, CLK_Pin);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();

 // HAL_ADC_Start_DMA(&hadc1, &readValue, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin, GPIO_PIN_RESET); // EN do MUX como 0
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(mode)
	  {
	  	 case 1:

	  		LCD_print("CalibrarPreto", 0, 0);
	  		HAL_Delay(5000);
	  		calibrarPreto();
	  		LCD_print("CalibrarBranco", 0, 0);
	  		HAL_Delay(5000);
	  		calibrarBranco();
	  		calcularMediaSensores();
  		    ligarMotorA();
  		    ligarMotorB();
	  		mode++;
	  		LCD_clrScr();
	  		LCD_print("Calibrado", 0, 0);
  			HAL_Delay(3000);
  			LCD_clrScr();
  			LCD_print("Andar", 0, 0);

	  	break;

	  	case 2:


  				sprintf(Buffer, "%d", error);
  				LCD_print(Buffer, 3, 3);
	  			aplicarCalibracao();
	  		    leituraLinha();
	  		    PID();
	  		    setPWM();







	  	break;
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4570;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4570;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_Pin|S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, S1_Pin|S2_Pin|S3_Pin|BL_Pin
                          |CLK_Pin|DIN_Pin|AI2_Pin|AI1_Pin
                          |STBY_Pin|BI1_Pin|BI2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DC_Pin|CE_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_Pin S0_Pin */
  GPIO_InitStruct.Pin = EN_Pin|S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin S3_Pin BL_Pin
                           CLK_Pin DIN_Pin AI2_Pin AI1_Pin
                           STBY_Pin BI1_Pin BI2_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin|S3_Pin|BL_Pin
                          |CLK_Pin|DIN_Pin|AI2_Pin|AI1_Pin
                          |STBY_Pin|BI1_Pin|BI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DC_Pin CE_Pin RST_Pin */
  GPIO_InitStruct.Pin = DC_Pin|CE_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BOT2_Pin BOT1_Pin */
  GPIO_InitStruct.Pin = BOT2_Pin|BOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
