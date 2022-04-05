/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

//For sprintf and memcpy
#include "string.h"
//For atoi
#include "stdlib.h"
//For ceil()
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//UJA Register Definitions

#define UJA_REG_WDGANDSTATUS 0
#define UJA_REG_MODECONTROL 1
#define UJA_REG_INTCON 2
#define UJA_REG_INTREAD 3

//WD_and_Status register
//RO
#define UJA_RO_RW 0
#define UJA_RO_R 1

//WMC
#define UJA_WMC_WND 0
#define UJA_WMC_TO 1

//NWP
#define UJA_NWP_8 0
#define UJA_NWP_16 1
#define UJA_NWP_32 2
#define UJA_NWP_64 3
#define UJA_NWP_128 4
#define UJA_NWP_256 5
#define UJA_NWP_1024 6
#define UJA_NWP_4096 7


//Mode_Control register
#define UJA_MC_STBY 0
#define UJA_MC_SLP 1
#define UJA_MC_V2OFF 2
#define UJA_MC_V2ON 3

//Int_Control register
#define UJA_V1UIE_OFF 0
#define UJA_V1UIE_ON 1

#define UJA_V2UIE_OFF 0
#define UJA_V2UIE_ON 1


//Message struct parameter
#define MSG_ENABLED 1
#define MSG_DISABLED 0


//CAN Output Frame bits
#define CAN_TXFRAME0_SHUTDOWN 1
#define CAN_TXFRAME0_BIT2 1<<1


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for ShiftCar */
osThreadId_t ShiftCarHandle;
const osThreadAttr_t ShiftCar_attributes = {
  .name = "ShiftCar",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for FeedWDG */
osThreadId_t FeedWDGHandle;
uint32_t FeedWDGBuffer[ 128 ];
osStaticThreadDef_t FeedWDGControlBlock;
const osThreadAttr_t FeedWDG_attributes = {
  .name = "FeedWDG",
  .stack_mem = &FeedWDGBuffer[0],
  .stack_size = sizeof(FeedWDGBuffer),
  .cb_mem = &FeedWDGControlBlock,
  .cb_size = sizeof(FeedWDGControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

struct message
{
	uint32_t id;
	uint8_t  bit;
	uint8_t length;
	int32_t value;
	uint8_t enabled;
};


//Create array of message structs to hold our filter information
struct message messageArray[4];

//Define starting filter array used on startup
const struct message defaultMessageArray[4] =
{
		//{ 0x370,   0,  16,   -1,  MSG_ENABLED  },  //Engine RPM
		{ 0x2C2,   0,   16,   -1,  MSG_ENABLED  },  //Upshift
		{ 0x2C2,  32,   16,   -1,  MSG_ENABLED  },  //Downshift
		{ 0x2C4,   0,   16,   -1,  MSG_ENABLED  },  //NeutralReq
		{ 0x2C6,   0,  32,   -1,  MSG_ENABLED  }   //Engine RPM
};


uint8_t can_rx_data[8];
CAN_RxHeaderTypeDef can_rx_header;

uint8_t can_tx_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
CAN_TxHeaderTypeDef can_tx_header = {0x200, 0, CAN_ID_STD, CAN_RTR_DATA, 8};

uint8_t uart_rec_buff[24];
char uart_tx_buff[128];

int8_t cmdcharbuffindex = 0;
uint8_t cmdbuff[20][24];
int8_t cmdbuffindex = 0;

uint8_t processingcommand = 0;
uint8_t cmdlen = 0;

const uint8_t WD_SETUP = (UJA_REG_WDGANDSTATUS << 5) | (UJA_RO_RW << 4) | (UJA_WMC_WND << 3) | (UJA_NWP_1024);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void StartShiftCar(void *argument);
void StartFeedWDG(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

void DebugPrint(char *msg);
void Init_SBC(void);
void ConfigureCANFilters(struct message * messageArray, uint8_t size);
void Init_CAN(void);
void ClearCANBuffers(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Prints a message to UART1 for debugging


//Initializes SBC
void Init_SBC(void)
{
	uint8_t txdata[2];
	uint8_t rxdata[2];

	//Force SBC in standby mode
	txdata[0] = (UJA_REG_MODECONTROL << 5) | (UJA_RO_RW << 4) | (UJA_MC_STBY << 2);
	txdata[1] = 0;
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 2, 100);
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 1);

	//Setup WDG and Status register
	txdata[0] = WD_SETUP;
	txdata[1] = 0;
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 2, 100);
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 1);

	//Set normal mode and enable CAN voltage
	txdata[0] = (UJA_REG_MODECONTROL << 5) | (UJA_RO_RW << 4) | (UJA_MC_V2ON << 2);
	txdata[1] = 0;
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 2, 100);
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 1);
}
/// @brief  Possible STM32 system reset causes
typedef enum reset_cause_e
{
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_LOW_POWER_RESET,
    RESET_CAUSE_WINDOW_WATCHDOG_RESET,
    RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET,
    RESET_CAUSE_SOFTWARE_RESET,
    RESET_CAUSE_POWER_ON_POWER_DOWN_RESET,
    RESET_CAUSE_EXTERNAL_RESET_PIN_RESET,
    RESET_CAUSE_BROWNOUT_RESET,
} reset_cause_t;

/// @brief      Obtain the STM32 system reset cause
/// @param      None
/// @return     The system reset cause
reset_cause_t reset_cause_get(void)
{
    reset_cause_t reset_cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_cause = RESET_CAUSE_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_cause = RESET_CAUSE_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_cause = RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        // This reset is induced by calling the ARM CMSIS
        // `NVIC_SystemReset()` function!
        reset_cause = RESET_CAUSE_SOFTWARE_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = RESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_cause = RESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
    }
    // Needs to come *after* checking the `RCC_FLAG_PORRST` flag in order to
    // ensure first that the reset cause is NOT a POR/PDR reset. See note
    // below.
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = RESET_CAUSE_BROWNOUT_RESET;
    }
    else
    {
        reset_cause = RESET_CAUSE_UNKNOWN;
    }

    // Clear all the reset flags or else they will remain set during future
    // resets until system power is fully removed.
    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause;
}

// Note: any of the STM32 Hardware Abstraction Layer (HAL) Reset and Clock
// Controller (RCC) header files, such as
// "STM32Cube_FW_F7_V1.12.0/Drivers/STM32F7xx_HAL_Driver/Inc/stm32f7xx_hal_rcc.h",
// "STM32Cube_FW_F2_V1.7.0/Drivers/STM32F2xx_HAL_Driver/Inc/stm32f2xx_hal_rcc.h",
// etc., indicate that the brownout flag, `RCC_FLAG_BORRST`, will be set in
// the event of a "POR/PDR or BOR reset". This means that a Power-On Reset
// (POR), Power-Down Reset (PDR), OR Brownout Reset (BOR) will trip this flag.
// See the doxygen just above their definition for the
// `__HAL_RCC_GET_FLAG()` macro to see this:
//      "@arg RCC_FLAG_BORRST: POR/PDR or BOR reset." <== indicates the Brownout
//      Reset flag will *also* be set in the event of a POR/PDR.
// Therefore, you must check the Brownout Reset flag, `RCC_FLAG_BORRST`, *after*
// first checking the `RCC_FLAG_PORRST` flag in order to ensure first that the
// reset cause is NOT a POR/PDR reset.


/// @brief      Obtain the system reset cause as an ASCII-printable name string
///             from a reset cause type
/// @param[in]  reset_cause     The previously-obtained system reset cause
/// @return     A null-terminated ASCII name string describing the system
///             reset cause
const char * reset_cause_get_name(reset_cause_t reset_cause)
{
    const char * reset_cause_name = "TBD";

    switch (reset_cause)
    {
        case RESET_CAUSE_UNKNOWN:
            reset_cause_name = "UNKNOWN";
            break;
        case RESET_CAUSE_LOW_POWER_RESET:
            reset_cause_name = "LOW_POWER_RESET";
            break;
        case RESET_CAUSE_WINDOW_WATCHDOG_RESET:
            reset_cause_name = "WINDOW_WATCHDOG_RESET";
            break;
        case RESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
            reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
            break;
        case RESET_CAUSE_SOFTWARE_RESET:
            reset_cause_name = "SOFTWARE_RESET";
            break;
        case RESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
            reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
            break;
        case RESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
            reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
            break;
        case RESET_CAUSE_BROWNOUT_RESET:
            reset_cause_name = "BROWNOUT_RESET (BOR)";
            break;
    }

    return reset_cause_name;
}


//Procedurally generates and sets CAN Filter configurations from the message[] struct array config
void ConfigureCANFilters(struct message * messageArray, uint8_t size)
{
	uint32_t configuredIDs[size];
	int uniques = 0;
	for (int i = 0; i < size; i++)
	{
		struct message thismessage = messageArray[i];

		//Check if this ID already configured
		int create = 1;
		for (int j = 0; j < size; j++)
		{
			if (configuredIDs[j] == thismessage.id)
			{
				create = 0;
			}
		}
		if (create == 1 && thismessage.enabled)
		{
			//Add this ID to the list of already configured ID's to skip duplicates
			configuredIDs[uniques] = thismessage.id;

			CAN_FilterTypeDef filter;

			//This bit shifting was a massive PITA to figure out... see page 1092 of the RM for reasoning
			filter.FilterIdHigh = ((thismessage.id << 5)  | (thismessage.id >> (32 - 5))) & 0xFFFF;
			filter.FilterIdLow = (thismessage.id >> (11 - 3)) & 0xFFF8;

			//Masks set to full rank to check every bit against ID
			filter.FilterMaskIdHigh = 0xFFFF;
			filter.FilterMaskIdLow = 0xFFFF;

			//Filter options
			filter.FilterScale = CAN_FILTERSCALE_32BIT;
			filter.FilterActivation = ENABLE;
			filter.FilterMode = CAN_FILTERMODE_IDMASK;
			filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;

			//Set filter bank to the current count of uniques
			filter.FilterBank = uniques;

			//Finally pass filter to HAL
			HAL_CAN_ConfigFilter(&hcan, &filter);
			uniques++;
		}
	}
}

void ClearCANBuffers(void)
{
	for (int i = 0; i < 16; i++) {
		messageArray[i].value = -1;
	}
}

void Init_CAN(void)
{
	//Configure all receive filters from the config array
	ConfigureCANFilters(messageArray, sizeof(messageArray) / sizeof(struct message));

	//Start CAN operation
	HAL_CAN_Start(&hcan);

	//Enable Message Pending IRQ
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

	//Start receiving - don't think we need this here
	HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data);
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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
	HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 1);

	Init_SBC();


	//Load default message configuration
	//NOTE: This operation takes approximately 100ms!!!!!
	memcpy(&messageArray, &defaultMessageArray, sizeof(messageArray));

	//Initialize CAN and all filters
	Init_CAN();


  //Start receiving UART
	HAL_UART_Receive_IT(&huart2, uart_rec_buff, 1);

	reset_cause_t reset_cause = reset_cause_get();
	printf("The system reset cause is \"%s\"\n", reset_cause_get_name(reset_cause));
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of ShiftCar */
  ShiftCarHandle = osThreadNew(StartShiftCar, NULL, &ShiftCar_attributes);

  /* creation of FeedWDG */
  FeedWDGHandle = osThreadNew(StartFeedWDG, NULL, &FeedWDG_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN_RX0_IRQn);
  /* CAN_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN_RX1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN_RX1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UJA_CS_Pin|LD2_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|IN2_Pin|INH1_Pin|INH2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UJA_CS_Pin LD2_Pin LD1_Pin */
  GPIO_InitStruct.Pin = UJA_CS_Pin|LD2_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin INH1_Pin INH2_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|INH1_Pin|INH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//IRQ's
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Check if enter or cmdbuff reaches its limit (prevents overflow)
	if (uart_rec_buff[0] == *(uint8_t *)"\r" || cmdcharbuffindex > 23)
	{
		//Make sure theres more than just \r sent.
		if (cmdcharbuffindex != 0)
		{
			//Increase command buffer index
			cmdbuffindex++;

			//Limit number of commands to 16.
			if (cmdbuffindex > 16)
			{
				cmdbuffindex = 16;
			}
			//Pass command onto task

			//Reset char index
			cmdcharbuffindex = 0;
		}
	}
	else
	{
		//Put received char into buffer
		cmdbuff[cmdbuffindex][cmdcharbuffindex] = uart_rec_buff[0];
		//Increase char index
		cmdcharbuffindex++;
	}

	//Start receiving again
	HAL_UART_Receive_IT(huart, uart_rec_buff, 1);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//Get the received message.
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data);

	//Parse received bytes using message array
	for(int i=0; i < sizeof(messageArray) / sizeof(struct message); i++)
	{
		if(messageArray[i].id == can_rx_header.StdId)
		{

			//Calculate which byte position to start at
			int bytepos = messageArray[i].bit / 8;

			//Calculate number of bytes that need to be checked
			int bytes = ceil(messageArray[i].length / (float)8);

			uint32_t finalval = 0;
			int j = 0;

			//Iterate through all bytes that must be read for this message
			for (int b = bytepos; b < bytepos + bytes; b++)
			{
				uint8_t tempval;
				//If on last byte we may need to truncate unneeded parts dependent on length of data
				if (b == bytepos + bytes - 1)
				{
					//We need a left shift and a right shift to extract the bits we want
					uint8_t byteoffset = (messageArray[i].length - ((bytes-1) * 8));
					uint8_t leftshift = 8 - (messageArray[i].bit - (bytepos * 8)) - byteoffset;
					uint8_t rightshift = 8 - byteoffset;
					tempval = (can_rx_data[b] << leftshift) >>  rightshift;
				}
				else
				{
					//Use the whole byte
					tempval = can_rx_data[b];
				}
				//Calculate the size of the next data section to find the required shift
				int nextsize = messageArray[i].length - ((j+1) * 8);

				//Limit it to 0
				if (nextsize < 0)
				{
					nextsize = 0;
				}
				finalval += tempval << nextsize;
				j++;
			}

			//Finally set final value into message array struct
			messageArray[i].value = finalval;
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartShiftCar */
/**
  * @brief  Function implementing the ShiftCar thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartShiftCar */
void StartShiftCar(void *argument)
{
  /* USER CODE BEGIN 5 */
	int shiftvalue=0;
  /* Infinite loop */
  for(;;)
  {
    if (messageArray[0].value==64001 && messageArray[1].value == 1 && messageArray[2].value == 1 && shiftvalue==0)
    {
    	HAL_GPIO_WritePin(INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin (INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin (IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin (IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

    	HAL_Delay(30);

    	HAL_GPIO_WritePin (INH1_GPIO_Port, INH1_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin (INH2_GPIO_Port, INH2_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin (IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin (IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    	shiftvalue=1;
    }

    if (messageArray[0].value == 1 && messageArray[1].value ==64001 && messageArray[2].value == 1 && messageArray[3].value<=10500 && shiftvalue==0)
    {
    	HAL_GPIO_WritePin (INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin (INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);
    	HAL_GPIO_WritePin (IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin (IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);

    	HAL_Delay(30);

    	HAL_GPIO_WritePin (INH1_GPIO_Port, INH1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (INH2_GPIO_Port, INH2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    	shiftvalue=2;
    }

    if (messageArray[0].value ==1 && messageArray[1].value == 1 && messageArray[2].value == 64001 && messageArray[3].value<=7000 && shiftvalue==0)
    {
    	HAL_GPIO_WritePin (INH1_GPIO_Port, INH1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (INH2_GPIO_Port, INH2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin (IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);

		HAL_Delay(10);

		HAL_GPIO_WritePin (INH1_GPIO_Port, INH1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (INH2_GPIO_Port, INH2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin (IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    	shiftvalue=3;
    }

    if (messageArray[0].value ==1 && messageArray[1].value == 1 && messageArray[2].value == 1 && shiftvalue!=0)
    {
    	shiftvalue=0;
    }

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartFeedWDG */
/**
* @brief Function implementing the FeedWDG thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFeedWDG */
void StartFeedWDG(void *argument)
{
  /* USER CODE BEGIN StartFeedWDG */
  /* Infinite loop */
  for(;;)
  {
  	//600 milliseconds is in the lower end of the watchdog window as currently configured in the SBC.
  	osDelay(600);
    uint8_t rxdata[2];
    uint8_t txdata[2];

    txdata[0] = WD_SETUP;
		txdata[1] = 0;

		HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 0);
		HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 2, 100);
		HAL_GPIO_WritePin(UJA_CS_GPIO_Port, UJA_CS_Pin, 1);

		//Toggle LED
		HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
  }
  /* USER CODE END StartFeedWDG */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
