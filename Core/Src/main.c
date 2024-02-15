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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "mpu6050.h"
#include "bmp180.h"
//#include "DS3231.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// CAN constants
#define SERVER_CAN_ID 				0x01
#define SENSOR_NODE_CAN_ID   		0x20
#define CAN_PACKET_LEN    			8
#define START_MESSAGE_LEN 			1
#define STOP_MESSAGE_LEN 			1
#define CAN_SEND_DELAY				250 // in milliseconds

// i2c connection states
#define I2C_CONNECTED				1
#define I2C_DISCONNECTED			2

// config sensor buffer states
#define SENSOR_BUF_CREATED			1
#define SENSOR_BUF_NOT_CREATED		0


// sensor node data states
#define SENSOR_NODE_RECEIVE_STATE	0
#define SENSOR_NODE_TRANSMIT_STATE	1
#define SENSOR_NODE_CONFIG_STATE	2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

// i2c variables
I2C_HandleTypeDef 			hi2c1;

// CAN variables
CAN_HandleTypeDef 			hcan;
CAN_RxHeaderTypeDef   		rx_header;
uint8_t               		rx_data[CAN_PACKET_LEN];
uint8_t receive_state_packet[CAN_PACKET_LEN] = {0xaa,0xff,0xaa,0xff,0xaa,0xff,0xaa,0xff};
volatile uint8_t 			sensor_node_state = SENSOR_NODE_CONFIG_STATE;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

// functions for sending CAN packets/buffer
// --------------------------------------------------------------------------------------------------------

// send packet of size data_len over CAN
static void CAN_send_packet(	CAN_HandleTypeDef* hcan,
								CAN_TxHeaderTypeDef* TxHeader,
								uint8_t* TxData,
								uint8_t data_len	);

// send start packet over CAN
static void CAN_send_start(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TxHeader);

// send stop packet over CAN
static void CAN_send_stop(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TxHeader);

// send buffer of packets over CAN
static void CAN_send_buffer(	CAN_HandleTypeDef* hcan,
								CAN_TxHeaderTypeDef* TxHeader,
								uint8_t* buffer,
								uint8_t buffer_len	);


static void CAN_send_receive_state(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TxHeader);


static uint8_t configure_sensors(uint8_t** sensor_config_buf, size_t* sensor_config_buf_len);
// --------------------------------------------------------------------------------------------------------



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
  MX_CAN_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  // sensors
  //############################################################################################################

  // initialise i2c sensors
  uint8_t* sensor_config_buf;
  size_t sensor_config_buf_len = 0;
  if(configure_sensors(&sensor_config_buf, &sensor_config_buf_len) != SENSOR_BUF_CREATED)
  {
	  // terminate if config sensor buffer cannot be created
	  return 0;
  }

  //############################################################################################################

  // Start the CAN peripheral
  //############################################################################################################
  HAL_CAN_Start(&hcan);
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }
  CAN_TxHeaderTypeDef TxHeader;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = SENSOR_NODE_CAN_ID;
  TxHeader.ExtId = 0;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
  //############################################################################################################

  // set up test data
  // ############################################################################################################



  // ############################################################################################################



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // check sensor node state
	  if( sensor_node_state == SENSOR_NODE_CONFIG_STATE )
	  {
		 CAN_send_packet(&hcan, &TxHeader, sensor_config_buf, sensor_config_buf_len);
		 HAL_Delay(CAN_SEND_DELAY);
	  }

	  else if( sensor_node_state == SENSOR_NODE_TRANSMIT_STATE )
	  {

	  }

	  else if( sensor_node_state == SENSOR_NODE_RECEIVE_STATE )
	  {
		  CAN_send_receive_state(&hcan, &TxHeader);
		  HAL_Delay(CAN_SEND_DELAY);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 150;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // configure CAN filter to only accept commands from the sensor server
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterBank = (uint32_t)10;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = SERVER_CAN_ID<<5;
  canfilterconfig.FilterIdLow = SERVER_CAN_ID<<5;
  canfilterconfig.FilterMaskIdHigh = SERVER_CAN_ID<<5;
  canfilterconfig.FilterMaskIdLow = SERVER_CAN_ID<<5;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

// function definitions
//-------------------------------------------------------------------------------------------------------------------------

static void CAN_send_packet(	CAN_HandleTypeDef* hcan,
								CAN_TxHeaderTypeDef* TxHeader,
								uint8_t* TxData,
								uint8_t data_len	)
{
	uint32_t TxMailbox;

	HAL_StatusTypeDef CAN_status;

	TxHeader->DLC = data_len;

	CAN_status = HAL_CAN_AddTxMessage(hcan, TxHeader, TxData, &TxMailbox);

	// IMPORTANT - must wait for CAN bytes to be sent before sending next section of bytes
	while(HAL_CAN_IsTxMessagePending(hcan, TxMailbox));

	if (CAN_status != HAL_OK)
	{
	HAL_CAN_GetError(hcan);
	Error_Handler();
	}
}

static void CAN_send_start(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TxHeader)
{
	uint8_t TxData = 0xff;
	CAN_send_packet(hcan, TxHeader, &TxData, START_MESSAGE_LEN);
}

static void CAN_send_stop(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TxHeader)
{
	uint8_t TxData = 0xaa;
	CAN_send_packet(hcan, TxHeader, &TxData, STOP_MESSAGE_LEN);
}

static void CAN_send_buffer(	CAN_HandleTypeDef* hcan,
								CAN_TxHeaderTypeDef* TxHeader,
								uint8_t* buffer,
								uint8_t buffer_len)
{
	if( buffer_len < CAN_PACKET_LEN )
	{
		TxHeader->DLC = buffer_len;
	}
	else
	{
		TxHeader->DLC = CAN_PACKET_LEN;
	}

	// iterate through bytes of data and send over CAN
	CAN_send_start(hcan, TxHeader);

	for(int mem_offset = 0; mem_offset < buffer_len; mem_offset += CAN_PACKET_LEN)
	{
	  CAN_send_packet(hcan, TxHeader, buffer+mem_offset, TxHeader->DLC);
	}

	CAN_send_stop(hcan, TxHeader);
}

static void CAN_send_receive_state(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* TxHeader)
{
	CAN_send_packet(hcan, TxHeader, receive_state_packet, CAN_PACKET_LEN);
}


static uint8_t configure_sensors(uint8_t** sensor_config_buf, size_t* sensor_config_buf_len)
{
	// setup sensor config buffer
	size_t i_sensor_config_buf_len = 0;
	uint8_t* i_sensor_config_buf = (uint8_t*)calloc(i_sensor_config_buf_len, 0);

	// combined sensor data buffer
	size_t sensor_data_buf_len = 0;

	// initialise i2c sensors
	struct MPU6050 mpu6050;
	MPU6050Init(&hi2c1, &mpu6050, 1, 1, 1, 1);
	uint8_t mpu6050_state =  MPU6050_connection_state();

	if( mpu6050_state == MPU6050_NET_CODE )
	{
		i_sensor_config_buf_len++;
		i_sensor_config_buf = realloc( i_sensor_config_buf, i_sensor_config_buf_len * sizeof(uint8_t) );
		i_sensor_config_buf[i_sensor_config_buf_len-1] = MPU6050_NET_CODE;
		sensor_data_buf_len += sizeof(mpu6050_data);

	}

	BMP180_Init(&hi2c1);
	BMP180_SetOversampling(BMP180_ULTRA);
	BMP180_UpdateCalibrationData();
	uint8_t bmp180_state = BMP180_connection_state();

	if( bmp180_state == BMP180_NET_CODE )
	{
		i_sensor_config_buf_len++;
		i_sensor_config_buf = realloc( i_sensor_config_buf, i_sensor_config_buf_len * sizeof(uint8_t) );
		i_sensor_config_buf[i_sensor_config_buf_len-1] = BMP180_NET_CODE;
		sensor_data_buf_len += sizeof(bmp180_data);

	}

	if( i_sensor_config_buf_len < 1 )
	{
		free(i_sensor_config_buf);
		*sensor_config_buf_len = i_sensor_config_buf_len;
		return SENSOR_BUF_NOT_CREATED;
	}
	else
	{
		*sensor_config_buf = (uint8_t*)calloc(i_sensor_config_buf_len, 0);
		*sensor_config_buf_len = i_sensor_config_buf_len;
		memcpy(*sensor_config_buf, i_sensor_config_buf, i_sensor_config_buf_len * sizeof(uint8_t));
		free(i_sensor_config_buf);
		return SENSOR_BUF_CREATED;
	}

}

//-------------------------------------------------------------------------------------------------------------------------



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
	  if( rx_header.StdId == SERVER_CAN_ID) // check for messages from central server
	  {
		  if( rx_data[0] == SENSOR_NODE_CAN_ID )
		  {
			  sensor_node_state = SENSOR_NODE_TRANSMIT_STATE;
		  }
		  else
		  {
			  sensor_node_state = SENSOR_NODE_RECEIVE_STATE;
		  }

	  }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{

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
  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
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
