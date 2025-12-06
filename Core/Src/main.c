/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "usbd_hid.h"
#include "stdio.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern USBD_HandleTypeDef hUsbDeviceFS;
typedef struct {
<<<<<<< HEAD
	uint8_t reportId;
	uint8_t buttonStatus;
    int16_t xMovement;
    int16_t yMovement;
    int8_t scroll;
} mouseReport;

typedef struct {
	uint8_t reportId; // Always 2
	uint8_t modifiers;
	uint8_t reserved;
	uint8_t key1;
	uint8_t key2;
	uint8_t key3;
	uint8_t key4;
	uint8_t key5;
	uint8_t key6;
}keyboardReport;

mouseReport reportData = {0x01, 0, 0, 0, 0};
keyboardReport reportData2 = {0x02, 0, 0, 0, 0, 0, 0, 0, 0};

void sendReportMouse(uint8_t buttons, int8_t x, int8_t y, int8_t scroll) {


	reportData.buttonStatus = buttons;
	reportData.xMovement = x;
	reportData.yMovement = y;
	reportData.scroll = scroll;


    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reportData, sizeof(mouseReport));
}

void sendReportKey(uint8_t key){
	reportData2.modifiers = 0x0;
	reportData2.key1 = key;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&reportData2, sizeof(keyboardReport));
}




=======
    uint8_t reportID;      // The ID specified in the Report Descriptor for the mouse (e.g., 0x02)
    uint8_t buttonStatus;  // Bitfield for Left, Right, Middle buttons
    int8_t xMovement;      // Signed 8-bit relative movement (change since last report)
    int8_t yMovement;// Signed 8-bit relative movement (change since last report)
    int8_t scroll;
} mouseReport;

void sendMouseMovement(int8_t x, int8_t y, uint8_t buttons) {
    mouseReport report_data;

    report_data.reportID = 0x02; // Use your actual Mouse Report ID
    report_data.buttonStatus = buttons;
    report_data.xMovement = x;
    report_data.yMovement = y;
    report_data.scroll = 0;

    // This function requires the extern-declared hUsbDeviceFS handle
    USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report_data, sizeof(mouseReport));
}
>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
<<<<<<< HEAD
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
=======
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e
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
  MX_DMA_Init();
<<<<<<< HEAD
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_init();
  extern float Gy, Gz;
  int buttonStatus = 0;


  int scrollStatus = 0;
  int prevKeyStatus = 0;
=======
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_init();
  extern float Gy, Gz;
  float sens = 5;
  float deadZone = 0.5;



>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MPU6050_Read_Values();
<<<<<<< HEAD
	  HAL_ADC_Start(&hadc1);
	  int leftClickStatus = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
	  int bKeyStatus = HAL_GPIO_ReadPin(GPIOB ,GPIO_PIN_10);
	  int precisionMode = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	  int reset = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
	  int quickMode = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	  int scroll = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	  int escKeyStatus = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	  float sensitivity = HAL_ADC_GetValue(&hadc1);
	  float sens = 3 + ((sensitivity/4096)*10);

	  float Gz1 = Gz*10 / sens;
	  float Gy1 = Gy*10 / sens;


	  if(leftClickStatus == 0){
		 buttonStatus = 0x01;
	  } else {
		  buttonStatus = 0x00;
	  }

	  if(scroll == 0){
		  scrollStatus = 2;
	  } else {
		  scrollStatus = 0;
	  }

	  if(precisionMode == 0){
		  Gy1 = Gy1*0.5;
		  Gz1 = Gz1*0.5;
	  }
	  else if(reset == 0){
		  Gy1 = 0;
		  Gz1 = 0;
	  }
	  else if(quickMode == 0){
		  Gy1 = 2*Gy1;
		  Gz1 = 2*Gz1;
	  }
	  else {
		  Gy1 = Gy1;
		  Gz1 = Gz1;
	  }




	  int currentKeyToPress = 0; // Default to "Release All"
	  if (bKeyStatus == 0){
		  currentKeyToPress = 0x05;
	  }
	  else if (escKeyStatus == 0) {
		  currentKeyToPress = 0x29;
	  }

	  // 2. Send Key Report ONLY if the state changed (Press or Release)
	  if (currentKeyToPress != prevKeyStatus) {
	      sendReportKey(currentKeyToPress); // This handles both Press AND Release (0)
	      prevKeyStatus = currentKeyToPress;
	  }



	  sendReportMouse(buttonStatus, -Gz1, Gy1, scrollStatus);
=======

	  float current_Gz = Gz / sens;
	  float current_Gy = Gy / sens;
	  if (current_Gz <= deadZone && current_Gz >= -deadZone){
		  current_Gz = 0;
	  }
	  if (current_Gy <= deadZone && current_Gy >= -deadZone){
		  current_Gy = 0;
	  	  }

	  sendMouseMovement(-current_Gz, current_Gy, 0);
>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e



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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
<<<<<<< HEAD
  sConfig.Channel = ADC_CHANNEL_1;
=======
  sConfig.Channel = ADC_CHANNEL_7;
>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
<<<<<<< HEAD
  GPIO_InitTypeDef GPIO_InitStruct = {0};
=======
>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

<<<<<<< HEAD
  /*Configure GPIO pins : PA5 PA6 PA7 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

=======
>>>>>>> afb0988ffc1b74f9c4f10309fe57bcda370fa95e
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
#ifdef USE_FULL_ASSERT
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
