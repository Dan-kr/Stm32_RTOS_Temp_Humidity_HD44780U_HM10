
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "stdio.h"
#include "string.h"
#include "uart2_write.h"
#include "i2c_lcd.h"
#include "float.h"
#include "semphr.h"

#define SHT30_ADDRESS 0x88  // SHT30 I2C Address (0x44)

// Custom data type declared to store data from sensor

typedef struct
{
float temperature;
float humidity;
}SensorData_t;


I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
I2C_LCD_HandleTypeDef lcd1;

//Define Task Handles
SensorData_t SharedData;
osThreadId_t sensorTaskHandle;
osThreadId_t BluetoothTaskHandle;
osThreadId_t USARTTaskHandle;
osThreadId_t DisplayTaskHandle;



/* function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);

void Bluetooth_Task(void *pvParameters);
void I2C_Task(void *pvParameters);
void SHT30_Task(void *pvParameters);
void USART_Task(void *pvParameters);
void SHT30_Read(float*temperature, float*humidity);




int main(void)
{

  /* MCU Configuration*/

  HAL_Init();

 
  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
 

//LCD Init
  lcd1.hi2c1 = &hi2c1;
  lcd1.address = 0x4E;
  lcd_init(&lcd1);



// Task attributes (stack size and priority)
const osThreadAttr_t sensorTask_attr = {
  .name = "SensorTask",
  .stack_size = 256 * 4,  // Stack size in bytes
  .priority = osPriorityNormal
};

const osThreadAttr_t bluetoothTask_attr = {
  .name = "BluetoothTask",
  .stack_size = 256 * 4,
  .priority = osPriorityLow
};

const osThreadAttr_t usartTask_attr = {
  .name = "USARTTask",
  .stack_size = 256 * 4,
  .priority = osPriorityLow
};

const osThreadAttr_t i2cTask_attr = {
  .name = "I2CTask",
  .stack_size = 256 * 4,
  .priority = osPriorityLow
};

  /* Init scheduler */
  osKernelInitialize();

  // Create Threads
  sensorTaskHandle = osThreadNew(SHT30_Task, NULL,&sensorTask_attr);
  BluetoothTaskHandle = osThreadNew(Bluetooth_Task, NULL,&bluetoothTask_attr);
  USARTTaskHandle= osThreadNew(USART_Task, NULL,&usartTask_attr);
  DisplayTaskHandle = osThreadNew(I2C_Task, NULL,&i2cTask_attr);


  /* Start scheduler */
  osKernelStart();

  
 
  while (1)
  {
 
  }
  
}

// Sersor Read function

void SHT30_Read(float*temperature, float*humidity) {

    uint8_t cmd[2] = {0x2C, 0x06};
    uint8_t data[6];


    //Send measurement command
    HAL_I2C_Master_Transmit(&hi2c2, SHT30_ADDRESS, cmd, 2, HAL_MAX_DELAY);
     // Wait for measurement to complete (15ms)
    HAL_Delay(20);
    // Read data
    HAL_I2C_Master_Receive(&hi2c2, SHT30_ADDRESS, data, 6, HAL_MAX_DELAY);
    

    // Convert raw values
    uint16_t rawTemp = (data[0] << 8) | data[1];
    uint16_t rawHum = (data[3] << 8) | data[4];
    //Convert to temperature and humidity
    SharedData.temperature = -45.0f + (175.0f *(float)rawTemp / 65535.0f);
    SharedData.humidity = 100.0f * (float)rawHum / 65535.0f;

}

//Sensor Task

void SHT30_Task(void *argument)
{

  while (1)
  
{
  // Read Sensor Data
  SHT30_Read(&SharedData.temperature,&SharedData.humidity);

  //Notify other tasks
  osThreadFlagsSet(BluetoothTaskHandle,0x01);
  osThreadFlagsSet(USARTTaskHandle,0x01);
  osThreadFlagsSet(DisplayTaskHandle,0x01);
  osDelay(3000);

}

}

//Bluetooth Task

void Bluetooth_Task(void *argument)
{
char Temperature[50];
char Humidity[50];
 
while (1)
{

// Wait for notification 
osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); 

//Sending data to HM10 mudule via usart1
sprintf(Temperature,"Tmp=%.2fC\r\n", SharedData.temperature);
sprintf(Humidity,"Hum=%.2f%%\r\n", SharedData.humidity-7);
HAL_UART_Transmit(&huart1, (uint8_t*)Temperature, strlen(Temperature), HAL_MAX_DELAY);
osDelay (20);
HAL_UART_Transmit(&huart1, (uint8_t*)Humidity, strlen(Humidity), HAL_MAX_DELAY);

}


}


// USART Task

void USART_Task(void *argument) {
  char message[50];

  while (1) {
    // Wait for notification 
    osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);

    // Read shared data
    sprintf(message, "Temp=%.2fC, Hum=%.2f%%\r\n", SharedData.temperature, SharedData.humidity-7);
    HAL_UART_Transmit(&huart2, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
  }
}

// LCD Display Task

void I2C_Task(void *argument) {
  char Temperature[100];
  char Humidity[100];

  // Convert float to string
  sprintf(Temperature, "Temp %.2f", SharedData.temperature);
  sprintf(Humidity, "Humidity %.2f%%", SharedData.humidity-7);

  while (1) {
  // Wait for notification 
  osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever); 
  //Print Data to LCD
  lcd_clear(&lcd1);
  lcd_puts(&lcd1,Temperature);
  lcd_gotoxy(&lcd1, 0, 1);
  lcd_puts(&lcd1,Humidity);

}
}


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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