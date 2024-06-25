/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "motor.h"
#include "lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
#define ARR_CNT 5
#define CMD_SIZE 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

unsigned RGB_Table[8][3] = {
  //  Red    Green  Blue

  {   0,   0,    0}, // off
  {   0,   0,  255}, // Blue
  { 163, 191,   64}, // yellow
  { 255,  69,    0}, // orange
  { 255,   0,    0}, // Red
  { 255, 255,  255}, // white  { 233, 151, 157},
  {   0, 255,  255}, // green
  { 255,   0,  255}, // pupple(magenta)
};
unsigned long Font[10] = {0X0FC, 0X018, 0X16C, 0X13C,
                          0X198, 0X1B4, 0X1F4, 0X01C,
                                        0X1FC, 0X1BC};

volatile unsigned char arrayNum[3]={0};

int m_dutyrate_cnt = 0;
unsigned int m_cnt = 0;

uint8_t rx2char;
uint8_t btchar;

volatile char rx2Data[50];
volatile char btData[50];

char rhtbuf[20];
char dustbuf[25];

volatile unsigned char btFlag = 0;
volatile unsigned char rx2Flag = 0;
volatile unsigned char m_cntFlag = 0;
volatile unsigned char powerFlag = 0;
volatile unsigned char power_modeFlag = 0;
volatile unsigned char a_mFlag = 1;
volatile unsigned char modeFlag = 1;
volatile unsigned char lcd_dustFlag = 0;
volatile unsigned char rhtFlag = 1;
volatile unsigned char dustFlag = 1;
volatile int fndFlag = 1;

void bluetooth_Event();
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void RGB(unsigned int n);
void display_fnd( int N );
void display_digit(int pos,int num);
void Flaginit();
void Poweroff();
void Poweron();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/*
	void EXTI12_IRQHandler(void)
	{
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
	}
	void EXTI13_IRQHandler(void)
	{
	  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
	}
	void EXTI14_IRQHandler(void)
	{
	   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
	}
	void EXTI15_IRQHandler(void)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
	}
	 */

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
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart4, &btchar,1);
  HAL_UART_Receive_IT(&huart2, &rx2char,1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_Base_Start_IT(&htim7);

  LCD_init();
  disp_ON_OFF(ON,OFF,OFF);

  RGB(0);

  printf("start main2()\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	  if(rx2Flag)
		  {
			printf("recv2 : %s\r\n",rx2Data);
			rx2Flag =0;
		  }

	    if(btFlag)
		  {
			btFlag =0;
			bluetooth_Event();
		  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 254;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}



/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA8
                           PA9 PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void MX_GPIO_LED_ON(int flag)
{
	HAL_GPIO_WritePin(GPIOC, flag, GPIO_PIN_SET);

}
void MX_GPIO_LED_OFF(int flag)
{
	HAL_GPIO_WritePin(GPIOC, flag, GPIO_PIN_RESET);
}
void MOTOR(unsigned int m_dutyrate)
{

		TIM10-> CCR1 = (int)(17700 * (m_dutyrate / 100.0 ));

}
void RGB(unsigned int n)
{
	if ( n == 0 )
	{
		htim1.Instance->CCR1 = RGB_Table[0][0]; //
		htim1.Instance->CCR2 = RGB_Table[0][1]; //
		htim1.Instance->CCR3 = RGB_Table[0][2]; //
	}
	else if(n == 1)
	{
		htim1.Instance->CCR1 = RGB_Table[1][0]; // blue
		htim1.Instance->CCR2 = RGB_Table[1][1]; // blue
		htim1.Instance->CCR3 = RGB_Table[1][2]; // blue
	}
	else if(n == 2)
	{
		htim1.Instance->CCR1 = RGB_Table[2][0]; // Green
		htim1.Instance->CCR2 = RGB_Table[2][1]; // Green
		htim1.Instance->CCR3 = RGB_Table[2][2]; // Green
	}
	else if(n == 3)
	{
		htim1.Instance->CCR1 = RGB_Table[3][0]; // Orange
		htim1.Instance->CCR2 = RGB_Table[3][1]; // Orange
		htim1.Instance->CCR3 = RGB_Table[3][2]; // Orange
	}
	else if(n == 4)
	{
		htim1.Instance->CCR1 = RGB_Table[4][0]; // Red
		htim1.Instance->CCR2 = RGB_Table[4][1]; // Red
		htim1.Instance->CCR3 = RGB_Table[4][2]; // Red
	}
	else if(n == 5)
	{
		htim1.Instance->CCR1 = RGB_Table[5][0]; // White
		htim1.Instance->CCR2 = RGB_Table[5][1]; // White
		htim1.Instance->CCR3 = RGB_Table[5][2]; // White
	}
	else if(n == 6)
	{
		htim1.Instance->CCR1 = RGB_Table[6][0]; // Green
		htim1.Instance->CCR2 = RGB_Table[6][1]; // Green
		htim1.Instance->CCR3 = RGB_Table[6][2]; // Green
	}
	else if(n == 7)
	{
		htim1.Instance->CCR1 = RGB_Table[7][0]; // Pupple
		htim1.Instance->CCR2 = RGB_Table[7][1]; // Pupple
		htim1.Instance->CCR3 = RGB_Table[7][2]; // Pupple
	}
}

void display_digit(int pos,int num)
{

//    GPIO_Write(GPIOC,GPIO_ReadInputData(GPIOC) | 0x0f00); //fnd all off
//    GPIO_Write(GPIOC,GPIO_ReadInputData(GPIOC) & ~(GPIO_Pin_8 << pos));
//    GPIO_Write(GPIOC,(GPIO_ReadInputData(GPIOC) & 0xff00 )| Font[num]);
	  GPIOC->ODR = ( GPIOC->ODR | 0x0e00); //fnd all off
	  GPIOC->ODR = ( GPIOC->ODR & ~(GPIO_PIN_9 << pos));
	  GPIOC->ODR = (( GPIOC->ODR & 0xfe03)| Font[num]);

}
void display_fnd( int N )  // Segment 함수 선언
{
  int Buff ;
  if(N < 0)
  {
    N=-N;
    arrayNum[0] = 0;
  }
  else
    arrayNum[0] = N /100;  // 세그먼트에서 사용하는 천의 자리를 추출

  Buff = N % 100 ;
  arrayNum[1] = Buff / 10 ; // 세그먼트에서 사용하는 백의자리 추출
  Buff = Buff % 10;
  arrayNum[2] = Buff;     // 세그먼트에서 사용하는 십의 자리 추출

}
void bluetooth_Event()
{

  int i=0;
  char * pToken;
  char tvalue[5];
  char rhvalue[5];
  char dustvalue[5];
  char temparray[10] = {0,};

  char * rht_subpArray2;
  char * dust_subpArray2;

  char * pArray[ARR_CNT]={0};
  char recvBuf[CMD_SIZE]={0};
  char sendBuf[CMD_SIZE]={0};

  strcpy(recvBuf,btData);

  printf("btData : %s\r\n",btData);

  pToken = strtok(recvBuf,"[@]");
  while(pToken != NULL)
  {
    pArray[i] =  pToken;
    if(++i >= ARR_CNT)
      break;
    pToken = strtok(NULL,"[@]");
  }

  if(dustFlag)
  {
  if(!strcmp(pArray[1],"DUST"))
    {
	  dustFlag = 1;
  	if(!strcmp(pArray[2],"BLUE"))
  	{
  		MOTOR(40);
  		RGB(1);
  		display_fnd(40);
  		sprintf(sendBuf,"[%s]%s\n",pArray[0],pArray[1]);
  	}
  	else if(!strcmp(pArray[2],"YELLOW"))
  	{
  		MOTOR(55);
  		RGB(2);
  		display_fnd(55);
  		sprintf(sendBuf,"[%s]%s\n",pArray[0],pArray[1]);
  	}
  	else if(!strcmp(pArray[2],"ORANGE"))
  	  	{
  			MOTOR(70);
  			RGB(3);
  			display_fnd(75);
  	  		sprintf(sendBuf,"[%s]%s\n",pArray[0],pArray[1]);
  	  	}
  	else if(!strcmp(pArray[2],"RED"))
  	  	{
  			MOTOR(80);
  			RGB(4);
  			display_fnd(95);
  	  		sprintf(sendBuf,"[%s]%s\n",pArray[0],pArray[1]);
  	  	}

    }
  }
  if(rhtFlag)
  {
  if(!strcmp(pArray[1],"RHT"))
  {
	  rht_subpArray2 = strtok(pArray[2], ".");
	  if(rht_subpArray2!=NULL)
	  {
		  strcpy(tvalue,rht_subpArray2);
		  rht_subpArray2 = strtok(NULL, ".");
		  if (rht_subpArray2 != NULL)
		    {
		       strcpy(rhvalue, rht_subpArray2);
		       sprintf(rhtbuf,"T: %sC RH: %sPer",tvalue,rhvalue);
		       lcd(0,1,"                ");
		       lcd(0,1,rhtbuf);
		       //printf("%s\r\n",tvalue);
		       //printf("%s\r\n",rhvalue);
		    }
	  }
   }
  }
  if(lcd_dustFlag)
  {
	  if(!strcmp(pArray[1],"DUST_VAL"))
      {
		  printf("%s\r\n",pArray[2]);
		  dust_subpArray2 = pArray[2];
		  strcpy(dustvalue, dust_subpArray2);
		  sprintf(dustbuf," Dust : %sug/m3",dustvalue);
    	  lcd(0,1,"                ");
    	  lcd(0,1,dustbuf);
    		       //printf("%s\r\n",tvalue);
    	  printf("%s\r\n",dustbuf);
      }
  }
  if(!strcmp(pArray[1],"LED"))
   {
 	if(!strcmp(pArray[2],"ON"))
 	{
 		MX_GPIO_LED_ON(0x00ff);
 		sprintf(sendBuf,"[%s]%s\n",pArray[0],pArray[1]);
 	}
 	else if(!strcmp(pArray[2],"OFF"))
 	{
 		MX_GPIO_LED_OFF(0x00ff);
 		sprintf(sendBuf,"[%s]%s\n",pArray[0],pArray[1]);
 	}

   }
  else if(!strncmp(pArray[1]," New conn",sizeof(" New conn")))
  {
      return;
  }
  else if(!strncmp(pArray[1]," Already log",sizeof(" Already log")))
  {
      return;
  }
  else
      return;

  sprintf(sendBuf,"[%s]%s@%s\n",pArray[0],pArray[1],pArray[2]);
  HAL_UART_Transmit(&huart4, (uint8_t *)sendBuf, strlen(sendBuf), 0xFFFF);

  // }

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
	   	static int i=0;
	   	rx2Data[i] = rx2char;
	   	if(rx2Data[i] == '\r')
	   	{
	   		rx2Data[i] = '\0';
	   		rx2Flag = 1;
	   		i = 0;
	   	}
	   	else
	   	{
	   		i++;
	   	}
	   	HAL_UART_Receive_IT(&huart2, &rx2char,1);
	}

	if(huart->Instance == UART4)
		{
			static int i=0;
			btData[i] = btchar;
			if(btData[i] == '\n')
			{
				btData[i] = '\0';
				if( powerFlag && !a_mFlag) btFlag = 1;
				i = 0;
			}
			else
			{
				i++;
			}
			HAL_UART_Receive_IT(&huart4, &btchar,1);
		}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_12)
  {
	  if(powerFlag)
	  {
		  Poweroff();
		  printf("Power OFF\r\n");
	  }
	  else
	  {
		  Poweron();
		  printf("Power ON\r\n");
	  }
  }
  else if(GPIO_Pin == GPIO_PIN_13)
	  {
	  	  power_modeFlag = 1;

		if(powerFlag && power_modeFlag && a_mFlag)
		 {
				  a_mFlag = 0;
				  MOTOR(0);
				  RGB(0);
				  display_fnd(0);
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
				  lcd(0,0,"                ");
				  lcd(0,0,"    AUTO MODE");
				  printf("Auto mode\r\n");

				  if(rhtFlag)
				  {
					  lcd(0,1,"                ");
					  lcd(0,1,rhtbuf);
				  }
				  if(lcd_dustFlag)
				  {
					  lcd(0,1,"                ");
					  lcd(0,1,dustbuf);
				  }
		 }
		 else if(powerFlag && !a_mFlag)
		 {

			 	 a_mFlag = 1;
			 	 dustFlag = 1;
			 	 modeFlag = 0;

				  MOTOR(0);
				  RGB(5);
				  display_fnd(0);
				  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
				  lcd(0,0,"                ");
				  lcd(0,0,"   MANUAL MODE");
				  printf("Manual mode\r\n");
				  lcd(0,1,"                ");
		 }

	  }
  else if(GPIO_Pin == GPIO_PIN_14)
 	  {
 		  if(powerFlag && a_mFlag && !modeFlag && !btFlag)
 		  {

 			 m_dutyrate_cnt += 10;
 			 if ( m_dutyrate_cnt > 100) m_dutyrate_cnt = 100;

 			 display_fnd(m_dutyrate_cnt);
 			 MOTOR(m_dutyrate_cnt);

			  if(m_dutyrate_cnt >= 0 && m_dutyrate_cnt <= 40)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,"Motor Speed Slow");
			  }
			  else if(m_dutyrate_cnt >= 41 && m_dutyrate_cnt <= 70)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,"Motor Speed Mid");
			  }
			  else if(m_dutyrate_cnt >= 71 && m_dutyrate_cnt <= 100)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,"Motor Speed High");
			  }

 			 printf("MOTOR : %d\r\n",m_dutyrate_cnt);

 		  }

 		  if(powerFlag && !a_mFlag && modeFlag)
 		  {
 			 modeFlag = 0;
 			 dustFlag = 0;
 			 printf("Sleep Mode\r\n");
 			 lcd(0,0,"                ");
 			 lcd(0,0,"   Sleep Mode");
 			 RGB(6);
 			 MOTOR(30);
 			 display_fnd(30);
			  if(rhtFlag)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,rhtbuf);
			  }
			  if(lcd_dustFlag)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,dustbuf);
			  }

 		  }
 		  else if(powerFlag && !a_mFlag && !modeFlag)
 		  {
  			 modeFlag = 1;
  			 printf("   Fast Mode\r\n");
  			 lcd(0,0,"                ");
  			 lcd(0,0,"   Fast Mode");
 			 RGB(7);
 			 MOTOR(95);
 			 display_fnd(95);
			  if(rhtFlag)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,rhtbuf);
			  }
			  if(lcd_dustFlag)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,dustbuf);
			  }
 		  }
 	  }
  else if(GPIO_Pin == GPIO_PIN_15)
   	  {
   		  if(powerFlag && a_mFlag && !modeFlag && !btFlag)
   		  {
   			 m_dutyrate_cnt -= 10;
			 if ( m_dutyrate_cnt < 0) m_dutyrate_cnt = 0;

			 display_fnd(m_dutyrate_cnt);
   			 MOTOR(m_dutyrate_cnt);

			  if(m_dutyrate_cnt >= 0 && m_dutyrate_cnt <= 40)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,"Motor Speed Slow");
			  }
			  else if(m_dutyrate_cnt >= 41 && m_dutyrate_cnt <= 70)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,"Motor Speed Middle");
			  }
			  else if(m_dutyrate_cnt >= 71 && m_dutyrate_cnt <= 100)
			  {
				  lcd(0,1,"                ");
				  lcd(0,1,"Motor Speed High");
			  }

   			 printf("MOTOR : %d\r\n",m_dutyrate_cnt);
   		  }
   		  else if(powerFlag && !a_mFlag)
   		  {
   			  if(rhtFlag)
   			  {
   				rhtFlag = 0;
   				lcd_dustFlag = 1;
   			  }
   			  else
   			  {
     			rhtFlag = 1;
     			lcd_dustFlag = 0;
   			  }
   		  }
   	  }
}

void Flaginit()
{
	powerFlag = 0;
	power_modeFlag = 0;
	a_mFlag = 1;
	modeFlag = 1;
	lcd_dustFlag = 0;
	rhtFlag = 1;
	dustFlag = 1;
}
void Poweroff()
{
	Flaginit();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
	RGB(0);
	MOTOR(0);
	GPIOC->ODR = ( GPIOC->ODR | 0x0e00);
	clrscr();
}
void Poweron()
{
	  powerFlag = 1;
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	  display_fnd(0);
	  clrscr();
	  lcd(0,0,"    Power ON");
	  lcd(0,1,"                ");
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7)
	{
		static int t_cnt = 0;
		static int digit = 0;
		t_cnt++;
		if(t_cnt >= 1000)	//1Sec
		{
			t_cnt = 0;
		    m_cnt++;
		    m_cntFlag = 1;
		}
		if(powerFlag)
		{
			display_digit(digit,arrayNum[digit]);
			digit++;
			if(digit == 3)
				digit = 0;
		}
	}
}

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
