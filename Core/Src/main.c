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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "stdio.h"
#include "string.h"

#include "mpu6050.h"
#include "i2c.h"
//#include <stdint.h>
//#include "math.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//#define RAD_TO_DEG 57.295779513082320876798154814105
//
//#define WHO_AM_I_REG 0x75
//#define PWR_MGMT_1_REG 0x6B
//#define SMPLRT_DIV_REG 0x19
//#define ACCEL_CONFIG_REG 0x1C
//#define ACCEL_XOUT_H_REG 0x3B
//#define TEMP_OUT_H_REG 0x41
//#define GYRO_CONFIG_REG 0x1B
//#define GYRO_XOUT_H_REG 0x43
//
//// Setup MPU6050
//#define MPU6050_ADDR 0xD0
//const uint16_t i2c_timeout = 100;
//const double Accel_Z_corrector = 14418.0;
//
//uint32_t timer;
//typedef struct {
//    double Q_angle;
//    double Q_bias;
//    double R_measure;
//    double angle;
//    double bias;
//    double P[2][2];
//} Kalman_t;
//
//
//Kalman_t KalmanX = {
//        .Q_angle = 0.001f,
//        .Q_bias = 0.003f,
//        .R_measure = 0.03f
//};
//
//Kalman_t KalmanY = {
//        .Q_angle = 0.001f,
//        .Q_bias = 0.003f,
//        .R_measure = 0.03f,
//};
//
//typedef struct {
//
//    int16_t Accel_X_RAW;
//    int16_t Accel_Y_RAW;
//    int16_t Accel_Z_RAW;
//    double Ax;
//    double Ay;
//    double Az;
//
//    int16_t Gyro_X_RAW;
//    int16_t Gyro_Y_RAW;
//    int16_t Gyro_Z_RAW;
//    double Gx;
//    double Gy;
//    double Gz;
//
//    float Temperature;
//
//    double KalmanAngleX;
//    double KalmanAngleY;
//    double yaw;
//} MPU6050_t;
//
MPU6050_t mpu6050_data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
xQueueHandle St_Queue_Handler;
//
//xTaskHandle Sender1_Task_Handler;
//xTaskHandle Sender2_Task_Handler;
xTaskHandle MPU6050ReadAll_Task_Handler;

xTaskHandle Receiver_Task_Handler;
//
//
//void Sender1_Task(void *argument);
//void Sender2_Task(void *argument);
void Receiver_Task(void *argument);
void MPU6050ReadAll_Task(void *argument);
/**************** STRUCTURE DEFINITION *****************/


int indx1 = 0;
int indx2 = 0;
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
  MPU6050_Init(&hi2c1);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
St_Queue_Handler = xQueueCreate(2, sizeof(MPU6050_t));
if (St_Queue_Handler == 0)	//Queue not created
{
	  char *str = "Unable to create Structured Queue\n\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
else
{
	  char *str = "Structured Queue created successfully\n\n";
	  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

//xTaskCreate(Sender1_Task, "SENDER1", 128, NULL, 2, &Sender1_Task_Handler);
//xTaskCreate(Sender2_Task, "SENDER2", 128, NULL, 2, &Sender2_Task_Handler);
xTaskCreate(Receiver_Task, "RECEIVER", 128, NULL, 1, &Receiver_Task_Handler);
xTaskCreate(MPU6050ReadAll_Task, "MPU6050READALL", 128, NULL, 2, &MPU6050ReadAll_Task_Handler);

vTaskStartScheduler();
  /* USER CODE END 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//uint8_t MPU6050Init_Task(I2C_HandleTypeDef *I2Cx) {
//    uint8_t check;
//    uint8_t Data;
//
//    // check device ID WHO_AM_I
//
//    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);
//
//    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
//    {
//        // power management register 0X6B we should write all 0's to wake the sensor up
//        Data = 0;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);
//
//        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
//        Data = 0x07;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);
//
//        // Set accelerometer configuration in ACCEL_CONFIG Register
//        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
//        Data = 0x00;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);
//
//        // Set Gyroscopic configuration in GYRO_CONFIG Register
//        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
//        Data = 0x00;
//        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
//        return 0;
//    }
//    return 1;
//}
void MPU6050ReadAll_Task (void *argument)
{
	uint32_t TickDelay = pdMS_TO_TICKS(2000);
	while(1)
	{
		mpu6050_data = MPU6050_Read_All();

		MPU6050_t *ptrtostruct;

		char *str = "Entered MPU6050READALL_Task\n about to SEND to the queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
		/****** ALOOCATE MEMORY TO THE PTR ********/
		ptrtostruct = pvPortMalloc(sizeof (MPU6050_t));
		/********** LOAD THE DATA ***********/
		ptrtostruct->Accel_X_RAW = mpu6050_data.Accel_X_RAW;
		ptrtostruct->Accel_Y_RAW = mpu6050_data.Accel_Y_RAW;
		ptrtostruct->Accel_Z_RAW = mpu6050_data.Accel_Z_RAW;

		ptrtostruct->Gyro_X_RAW = mpu6050_data.Gyro_X_RAW;
		ptrtostruct->Gyro_Y_RAW = mpu6050_data.Gyro_Y_RAW;
		ptrtostruct->Gyro_Z_RAW = mpu6050_data.Gyro_Z_RAW;

		ptrtostruct->Ax = mpu6050_data.Ax;
		ptrtostruct->Ay = mpu6050_data.Ay;
		ptrtostruct->Az = mpu6050_data.Az;

		ptrtostruct->Gx = mpu6050_data.Gx;
		ptrtostruct->Gy = mpu6050_data.Gy;
		ptrtostruct->Gz = mpu6050_data.Gz;

		/***** send to the queue ****/
		if (xQueueSend(St_Queue_Handler, &ptrtostruct, portMAX_DELAY) == pdPASS)
		{
		char *str2 = " Successfully sent the to the queue\nLeaving MPU6050ReadAll_Task\n\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
		}
//		indx1 = indx1+1;
		vTaskDelay(TickDelay);
	}
}
//void Sender1_Task (void *argument)
//{
////	MPU6050_t *ptrtostruct;
////	uint32_t TickDelay = pdMS_TO_TICKS(2000);
////	while (1)
////	{
////	char *str = "Entered SENDER1_Task\n about to SEND to the queue\n\n";
////	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
////	/****** ALOOCATE MEMORY TO THE PTR ********/
////	ptrtostruct = pvPortMalloc(sizeof (MPU6050_t));
////	/********** LOAD THE DATA ***********/
////	ptrtostruct->counter = 1+indx1;
////	ptrtostruct->large_value = 1000 + indx1*100;
////	ptrtostruct->str = "HELLO FROM SENDER 1 ";
////	/***** send to the queue ****/
////	if (xQueueSend(St_Queue_Handler, &ptrtostruct, portMAX_DELAY) == pdPASS)
////	{
////	char *str2 = " Successfully sent the to the queue\nLeaving SENDER1_Task\n\n\n";
////	HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
////	}
////	indx1 = indx1+1;
////	vTaskDelay(TickDelay);
////	}
//}

//void Sender2_Task (void *argument)
//{
////	MPU6050_t *ptrtostruct;
////	uint32_t TickDelay = pdMS_TO_TICKS(2000);
////	while (1)
////	{
////	char *str = "Entered SENDER2_Task\n about to SEND to the queue\n\n";
////	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
////	/****** ALOOCATE MEMORY TO THE PTR ********/
////	ptrtostruct = pvPortMalloc(sizeof (MPU6050_t));
////	/********** LOAD THE DATA ***********/
////	ptrtostruct->counter = 1+indx1;
////	ptrtostruct->large_value = 1000 + indx1*100;
////	ptrtostruct->str = "SENDER 2 says Hi!!! ";
////	/***** send to the queue ****/
////	if (xQueueSend(St_Queue_Handler, &ptrtostruct, portMAX_DELAY) == pdPASS)
////	{
////	char *str2 = " Successfully sent the to the queue\nLeaving SENDER2_Task\n\n\n";
////	HAL_UART_Transmit(&huart2, (uint8_t *)str2, strlen (str2), HAL_MAX_DELAY);
////	}
////	indx1 = indx1+1;
////	vTaskDelay(TickDelay);
////	}
//}

void Receiver_Task (void *argument)	//se la ham pid_compute()
{
	MPU6050_t *Rptrtostruct;
	uint32_t TickDelay = pdMS_TO_TICKS(3000);
	char *ptr;
	while (1)
	{
		char *str = "Entered RECEIVER Task\n about to RECEIVE FROM the queue\n\n";
		HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
		/**** RECEIVE FROM QUEUE *****/
		if (xQueueReceive(St_Queue_Handler, &Rptrtostruct, portMAX_DELAY) == pdPASS)
		{
			ptr = pvPortMalloc(100 * sizeof (char)); // allocate memory for the string
//			sprintf (ptr, "Received from QUEUE:\n Accel_X_RAW = %d\n Accel_Y_RAW = %d\n Accel_Z_RAW = %d\n\n\n",Rptrtostruct->Accel_X_RAW,Rptrtostruct->Accel_Y_RAW, Rptrtostruct->Accel_Z_RAW);
			sprintf (ptr, "Accel_X_RAW = %d\n Accel_Y_RAW = %d\n Accel_Z_RAW = %d\n\n\n",Rptrtostruct->Accel_X_RAW,Rptrtostruct->Accel_Y_RAW, Rptrtostruct->Accel_Z_RAW);
			HAL_UART_Transmit(&huart2, (uint8_t *)ptr, strlen(ptr), HAL_MAX_DELAY);
			vPortFree(ptr);  // free the string memory
		}
		vPortFree(Rptrtostruct);  // free the structure memory
		vTaskDelay(TickDelay);
	}
}

//void Receiver_Task (void *argument)
//{
//	my_struct *Rptrtostruct;
//	uint32_t TickDelay = pdMS_TO_TICKS(3000);
//	char *ptr;
//	while (1)
//	{
//	char *str = "Entered RECEIVER Task\n about to RECEIVE FROM the queue\n\n";
//	HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);
//	/**** RECEIVE FROM QUEUE *****/
//	if (xQueueReceive(St_Queue_Handler, &Rptrtostruct, portMAX_DELAY) == pdPASS)
//	{
//	ptr = pvPortMalloc(100 * sizeof (char)); // allocate memory for the string
//	sprintf (ptr, "Received from QUEUE:\n COUNTER = %d\n LARGE VALUE = %u\n STRING = %s\n\n\n",Rptrtostruct->counter,Rptrtostruct->large_value, Rptrtostruct->str);
//	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, strlen(ptr), HAL_MAX_DELAY);
//	vPortFree(ptr);  // free the string memory
//	}
//	vPortFree(Rptrtostruct);  // free the structure memory
//	vTaskDelay(TickDelay);
//	}
//}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void const * argument)
//{
//  /* USER CODE BEGIN 5 */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END 5 */
//}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
