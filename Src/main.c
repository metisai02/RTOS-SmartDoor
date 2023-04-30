/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vantay.h"
#include "rfid.h"
#include "lcd_i2c.h"
#include "keypad.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LCD_ADDR 0x27

uint8_t data_winform = 0;
uint32_t start1, end1, ticks;
uint8_t register_FingerResult = FP_NOFINGER;
uint8_t IDFromFinger;
uint8_t CurrentNumberFinger;

uint8_t ack_fp[] = "N";
uint8_t ack_rfid[] = "M";
uint8_t ack_pass[] = "H";

char password[4] = {'1', '2', '3', '4'};
char getPassword[4];
char getPassword_temp1[4];
char getPassword_temp2[4];

uchar UID_1[5], UID_2[5], UID_3[5];
uchar str[16];
char fp_str[17] = {'\0'};
char str2[17] = {'\0'};
char str_it[17] = {'\0'};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId KeyPadHandle;
osThreadId RFIDHandle;
osThreadId FingerPrinfHandle;
osThreadId INTERRUPTHandle;
osMessageQId RegisterHandle;
osSemaphoreId myBinarySem01Handle;
/* USER CODE BEGIN PV */
I2C_LCD_Handle_t LCD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const *argument);
void StartTask02(void const *argument);
void StartTask03(void const *argument);
void StartTask04(void const *argument);

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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Register */
  osMessageQDef(Register, 1, uint8_t);
  RegisterHandle = osMessageCreate(osMessageQ(Register), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of KeyPad */
  osThreadDef(KeyPad, StartDefaultTask, osPriorityBelowNormal, 0, 128);
  KeyPadHandle = osThreadCreate(osThread(KeyPad), NULL);

  /* definition and creation of RFID */
  osThreadDef(RFID, StartTask02, osPriorityNormal, 0, 128);
  RFIDHandle = osThreadCreate(osThread(RFID), NULL);

  /* definition and creation of FingerPrinf */
  osThreadDef(FingerPrinf, StartTask03, osPriorityNormal, 0, 128);
  FingerPrinfHandle = osThreadCreate(osThread(FingerPrinf), NULL);

  /* definition and creation of INTERRUPT */
  osThreadDef(INTERRUPT, StartTask04, osPriorityAboveNormal, 0, 256);
  INTERRUPTHandle = osThreadCreate(osThread(INTERRUPT), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  keypad_init();
  MFRC522_Init();
  I2C_LCD_init(&LCD, &hi2c1, LCD_ADDR);
  HAL_UART_Receive_IT(&huart3, &data_winform, 1);
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
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
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | MFRC522_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MFRC522_CS_GPIO_Port, MFRC522_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           MFRC522_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | MFRC522_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MFRC522_CS_Pin */
  GPIO_InitStruct.Pin = MFRC522_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MFRC522_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  if (huart->Instance == USART3)
  {
    xSemaphoreGiveFromISR(myBinarySem01Handle, &xHigherPriorityTaskWoken);
    xQueueSendFromISR(RegisterHandle, &data_winform, NULL);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the KEYPAD thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{

  /* USER CODE BEGIN 5 */
  char key;
  uint8_t count = 0;
  /* Infinite loop */
  for (;;)
  {
    key = get_char();

    if (key != '*' && key != '#' && key != 'D' && key != 0 && key != 0x01 && key != 'A' && key != 'B')
    {
      I2C_LCD_display_clear(&LCD);
      I2C_LCD_print_string(&LCD, "Press Ur Pass!");
      getPassword[count] = key;
      vTaskDelay(100);
      for (uint8_t i = 0; i <= count; i++)
      {
        I2C_LCD_set_cursor(&LCD, 1, i);
        I2C_LCD_print_char(&LCD, '*');
      }
      count++;
      if (count == 4)
      {
        if (getPassword[0] == password[0] && getPassword[1] == password[1] &&
            getPassword[2] == password[2] && getPassword[3] == password[3])
        {
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_set_cursor(&LCD, 0, 4);
          I2C_LCD_print_string(&LCD, "WELCOME!!");
          I2C_LCD_set_cursor(&LCD, 1, 4);
          I2C_LCD_print_string(&LCD, "WELCOME!!");
          vTaskDelay(2000);
          count = 0;
        }
        else
        {
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_set_cursor(&LCD, 0, 1);
          I2C_LCD_print_string(&LCD, "Wrong Password");
          I2C_LCD_set_cursor(&LCD, 1, 1);
          I2C_LCD_print_string(&LCD, "Wrong Password");
          vTaskDelay(1000);
          I2C_LCD_display_clear(&LCD);
          count = 0;
        }
      }
    }

    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the RFID thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void const *argument)
{
  /* USER CODE BEGIN StartTask02 */
  uint8_t status = MI_ERR;
  /* Infinite loop */
  for (;;)
  {
    status = MFRC522_Request(PICC_REQIDL, str);
    if (status == MI_OK)
    {
      status = MFRC522_Anticoll(str);
      if (status == MI_OK)
      {
        if ((UID_1[0] == str[0] && UID_1[1] == str[1] && UID_1[2] == str[2] && UID_1[3] == str[3] && UID_1[4] == str[4]) ||
            (UID_2[0] == str[0] && UID_2[1] == str[1] && UID_2[2] == str[2] && UID_2[3] == str[3] && UID_2[4] == str[4]) ||
            (UID_3[0] == str[0] && UID_3[1] == str[1] && UID_3[2] == str[2] && UID_3[3] == str[3] && UID_3[4] == str[4]))
        {
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_set_cursor(&LCD, 0, 4);
          I2C_LCD_print_string(&LCD, "WELCOME!!");
          I2C_LCD_set_cursor(&LCD, 1, 4);
          I2C_LCD_print_string(&LCD, "WELCOME!!");
          vTaskDelay(2000);
        }
        else
        {
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_set_cursor(&LCD, 0, 3);
          I2C_LCD_print_string(&LCD, "RFID Error!!");
          I2C_LCD_set_cursor(&LCD, 1, 3);
          I2C_LCD_print_string(&LCD, "RFID Error!!");
          vTaskDelay(1000);
        }
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the FingerPrinf thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint8_t Result = FP_NOFINGER;
  /* Infinite loop */
  for (;;)
  {
    SendFPHeader();
    SendFPGetImage();
    Result = CheckFPRespsone(12);

    if (Result == FP_OK)
    {
      SendFPHeader();
      SendFPCreateCharFile1();
      Result = CheckFPRespsone(12);
      if (Result == FP_OK)
      {
        SendFPHeader();
        SendFPDSearchFinger();
        Result = CheckFPRespsone(16);

        I2C_LCD_display_clear(&LCD);
        I2C_LCD_set_cursor(&LCD, 0, 4);
        I2C_LCD_print_string(&LCD, "WELCOME!!");
        I2C_LCD_set_cursor(&LCD, 1, 4);
        I2C_LCD_print_string(&LCD, "WELCOME!!");
        vTaskDelay(2000);
      }
    }
    else if (Result == FP_ERROR)
    {
      I2C_LCD_display_clear(&LCD);
      I2C_LCD_set_cursor(&LCD, 0, 1);
      I2C_LCD_print_string(&LCD, "Finger Error!!");
      I2C_LCD_set_cursor(&LCD, 1, 1);
      I2C_LCD_print_string(&LCD, "Finger Error!!");
      vTaskDelay(2000);
    }

    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the INTERRUPT thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void const *argument)
{
  /* USER CODE BEGIN StartTask04 */
  uint8_t check_data;
  uint8_t r_counter = 0;
  char rc_key = 0;
  uint8_t status = 0;
  /* Infinite loop */
  for (;;)
  {
    if (xSemaphoreTake(myBinarySem01Handle, portMAX_DELAY) == pdPASS)
    {
      if (xQueueReceive(RegisterHandle, &check_data, 10) == pdPASS)
      {
        if (check_data == 'A')
        {
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_print_string(&LCD, "Register Finger");
          CurrentNumberFinger = GetNumberOfFinger();
          if (CurrentNumberFinger > 255)
          {
            CurrentNumberFinger = 1;
          }
          I2C_LCD_set_cursor(&LCD, 1, 0);
          sprintf(fp_str, "Number ID: %d", CurrentNumberFinger);
          I2C_LCD_print_string(&LCD, fp_str);
          register_FingerResult = RegistryNewFinger(CurrentNumberFinger + 1);

          if (register_FingerResult == FP_NOFINGER)
          {
            I2C_LCD_display_clear(&LCD);
            I2C_LCD_print_string(&LCD, "Finger Not Found");
            vTaskDelay(500);
          }
          else if (register_FingerResult == FP_OK)
          {
            I2C_LCD_display_clear(&LCD);
            I2C_LCD_print_string(&LCD, "Registry Finger");
            I2C_LCD_set_cursor(&LCD, 1, 5);
            I2C_LCD_print_string(&LCD, "DONE :>");
            vTaskDelay(1000);
          }
          else
          {
            I2C_LCD_display_clear(&LCD);
            I2C_LCD_set_cursor(&LCD, 0, 1);
            I2C_LCD_print_string(&LCD, "Finger Error!!");
            I2C_LCD_set_cursor(&LCD, 1, 1);
            I2C_LCD_print_string(&LCD, "Finger Error!!");
            vTaskDelay(500);
          }
        }
        else if (check_data == 'B')
        {
          // RFID
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_set_cursor(&LCD, 0, 4);
          I2C_LCD_print_string(&LCD, "ADD RFID");
          HAL_Delay(700);
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_print_string(&LCD, "Choose The Card");
          I2C_LCD_set_cursor(&LCD, 1, 0);
          I2C_LCD_print_string(&LCD, "To Replace:1,2,3");
          do
          {
            while ((rc_key = get_char()) == 0)
              ;
            if (rc_key == '1' || rc_key == '2' || rc_key == '3')
            {
              r_counter++;
            }

          } while (r_counter < 1);
          r_counter = 0;
          sprintf(str_it, "Your number: %x", rc_key);
          I2C_LCD_display_clear(&LCD);
          I2C_LCD_print_string(&LCD, str_it);
          HAL_Delay(1000);

          I2C_LCD_display_clear(&LCD);
          I2C_LCD_set_cursor(&LCD, 0, 2);
          I2C_LCD_print_string(&LCD, "Put Ur Card!!");
          I2C_LCD_set_cursor(&LCD, 1, 2);
          I2C_LCD_print_string(&LCD, "Put Ur Card!!");
          while ((status = MFRC522_Request(PICC_REQIDL, str)) != MI_OK)
          {
          }
          status = MFRC522_Anticoll(str);
          if (status == MI_OK)
          {
            sprintf(str2, "UID_%x: %x,%x,%x,%x", rc_key, str[0], str[1], str[2], str[3]);
            if (rc_key == '1')
            {
              UID_1[0] = str[0];
              UID_1[1] = str[1];
              UID_1[2] = str[2];
              UID_1[3] = str[3];
              UID_1[4] = str[4];
            }
            else if (rc_key == '2')
            {
              UID_2[0] = str[0];
              UID_2[1] = str[1];
              UID_2[2] = str[2];
              UID_2[3] = str[3];
              UID_2[4] = str[4];
            }
            else
            {
              UID_3[0] = str[0];
              UID_3[1] = str[1];
              UID_3[2] = str[2];
              UID_3[3] = str[3];
              UID_3[4] = str[4];
            }
            I2C_LCD_display_clear(&LCD);
            I2C_LCD_set_cursor(&LCD, 1, 0);
            I2C_LCD_print_string(&LCD, str2);
            vTaskDelay(2000);
          }
          else
          {
            I2C_LCD_display_clear(&LCD);
            I2C_LCD_set_cursor(&LCD, 0, 3);
            I2C_LCD_print_string(&LCD, "RFID Error!!");
            I2C_LCD_set_cursor(&LCD, 1, 3);
            I2C_LCD_print_string(&LCD, "RFID Error!!");
            vTaskDelay(1000);
          }
          HAL_UART_Transmit(&huart3, ack_rfid, 1, 1000);
        }
      }
      HAL_UART_Receive_IT(&huart3, &data_winform, 1);
    }

    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
