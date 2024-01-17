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
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define id_tx 0x10 //MAESTRO
#define id_rx 0x123 //ESCLAVO Nosotros
#define DLCSENSOR 8
#define BROADCAST_MODE 1// Broadcast sensor Work mode
#define POLL_MODE 2 //Poll sensor Work mode

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef rxHeader;
CAN_TxHeaderTypeDef txHeader;

uint8_t CanTxBuffer[8]; //Buffer escritura
uint8_t CanRxBuffer[8]; //Buffer lectura
uint32_t CanMailBox; //Para enviar
uint8_t flag_rx = 0, tx_complete = 0, tx_retry = 0, read_complete = 0; // banderas
uint8_t counterTime = 5; // contador tiempo
uint8_t WorkMode = BROADCAST_MODE; // por default va a trabajar en broadcast mode
uint8_t CycleTime = 10, CycleSensor = 5; //Cada cuanto envia los datos en broadcast mode, por default 10ms
uint8_t valueCycleTime = 10;
uint8_t SensorData[8]; //Buffer Sensor
uint16_t ChangeValueSensorTime = 2000;
HAL_CAN_StateTypeDef status1;
//uint8_t count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Filter_Config(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void SendMessage();
void ChangeValueSensorData();
void ReadSensor();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// COMO ENVIA LOS DATOS EN SENSOR... DLC = 8 RTR = DATA IDE = STANDARD
// B1 B2 		B3 B4 		B5 B6 		B7 B8
// H  L  		H  L  		H  L  		H  L
// Right 		Middle 		Left 		Cells state

// 0.1mm/unit es relacion de los datos sensados, ejemplo Middle = 200 significa que hay 20mm de desviacion con respecto a middle
// 0x8000 valdra si no se detecta linea o cinta, para este caso Right y Left = 0x8000
// Cell states B8 tiene los sensores de la izquierda y B7 los de la derecha
// B0 B1 B2 B3 B4 B5 B6 B7				B0 B1 B2 B3 B4 B5 B6
// 			byte 8								byte 7


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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_TIM_Base_Start_IT(&htim2);

  txHeader.DLC = DLCSENSOR;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.StdId = id_tx; // DIRECCION A RESPONDER
/*
  SensorData[0] = 0x80;	//right			H		0x80 = 1000 0000 0000 0000
  SensorData[1] = 0;	//right			L
  SensorData[2] = 0;	//middle		H		0000 0000
  SensorData[3] = 200;	//middle		L		1100 1000
  SensorData[4] = 0x80;	//left			H		1000 0000
  SensorData[5] = 0;	//left			L
  SensorData[6] = 0x03;	//cell state	H		 0000 011
  SensorData[7] = 0x80;	//cell state	L		1000 0000
*/

  //Para simular el envio de los estados de los sensores, definimos usar el byte 07 para los 4 sensores de la derecha (sensores 0,1,2,3)
  //y el byte 08 para los 4 de la izquierda (sensores 4,5,6,7)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 if(!CycleSensor){
		 ReadSensor();
		 CycleSensor = 5;
	 }

	 if(!CycleTime || tx_retry){
		 SendMessage();
		 HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);

	 }
	 if(tx_complete){
		 CycleTime = valueCycleTime;
		 //ChangeValueSensorData();
		 tx_complete = 0;
	 }
	 /*
	 if(!ChangeValueSensorTime){
		 ChangeValueSensorData();
		 HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		 HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
		 ChangeValueSensorTime = 2000;
	 }*/
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

/* USER CODE BEGIN 4 */

/* FUNCION PARA SIMULAR SENSORES
void ChangeValueSensorData(){
	if (SensorData[3] <= 254){
		SensorData[3] += 1;
	}
	else {
		SensorData[3] = 0;
		if(SensorData[2] <= 254)
			SensorData[2] += 1;
		else
			SensorData[2] = 0;
	}

}
*/
void SendMessage(){
	for (uint8_t i = 0; i<8; i++)
		CanTxBuffer[i] = SensorData[i];

	tx_complete = 0;
	if(HAL_CAN_IsTxMessagePending(&hcan, CanMailBox) == 0){
		if (HAL_CAN_AddTxMessage(&hcan, &txHeader, CanTxBuffer, &CanMailBox) == HAL_OK){
			tx_complete = 1;
			read_complete = 0;
			tx_retry = 0;
		}
		else
			tx_retry = 1;
	}
	else
		tx_retry = 1;
}

void ReadSensor(){

	SensorData[6] = 0;
	SensorData[7] = 0;

	SensorData[6] |= HAL_GPIO_ReadPin(SENSOR_0_GPIO_Port, SENSOR_0_Pin);//enviamos el estado del sensor 0 en el bit 0
	SensorData[6] |= (HAL_GPIO_ReadPin(SENSOR_1_GPIO_Port, SENSOR_1_Pin)<<1);//enviamos el estado del sensor 1 en el bit 1
	SensorData[6] |= (HAL_GPIO_ReadPin(SENSOR_2_GPIO_Port, SENSOR_2_Pin)<<2);//enviamos el estado del sensor 2 en el bit 2
	SensorData[6] |= (HAL_GPIO_ReadPin(SENSOR_3_GPIO_Port, SENSOR_3_Pin)<<3);//enviamos el estado del sensor 3 en el bit 3

	SensorData[7] |= HAL_GPIO_ReadPin(SENSOR_4_GPIO_Port, SENSOR_4_Pin);
	SensorData[7] |= (HAL_GPIO_ReadPin(SENSOR_5_GPIO_Port, SENSOR_5_Pin)<<1);
	SensorData[7] |= (HAL_GPIO_ReadPin(SENSOR_6_GPIO_Port, SENSOR_6_Pin)<<2);
	SensorData[7] |= (HAL_GPIO_ReadPin(SENSOR_7_GPIO_Port, SENSOR_7_Pin)<<3);

	read_complete = 1;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(CycleTime){
		CycleTime--;
	}
	if(CycleSensor){
		CycleSensor--;
	}
	/*
	if(ChangeValueSensorTime){
		ChangeValueSensorTime--;
	}*/
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, CanRxBuffer) == HAL_OK){
		flag_rx = 1;
	}
}

void CAN_Filter_Config(void) //Funcion para filtro can
{
	CAN_FilterTypeDef canfilter;

	canfilter.FilterActivation = CAN_FILTER_ENABLE;
	canfilter.FilterBank = 10;
	canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilter.FilterIdHigh = (id_rx << 5);
	canfilter.FilterIdLow = 0x0000;
	canfilter.FilterMaskIdHigh = (id_rx << 5);
	canfilter.FilterMaskIdLow = 0x0000;
	canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilter.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(&hcan, &canfilter);
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
