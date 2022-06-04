/* USER CODE BEGIN Header */
<<<<<<< HEAD
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
=======
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
<<<<<<< HEAD
#include "CONFIG.h"

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

=======
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
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
<<<<<<< HEAD
 TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for Task01 */
osThreadId_t Task01Handle;
const osThreadAttr_t Task01_attributes = {
  .name = "Task01",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task03 */
osThreadId_t Task03Handle;
const osThreadAttr_t Task03_attributes = {
  .name = "Task03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task04 */
osThreadId_t Task04Handle;
const osThreadAttr_t Task04_attributes = {
  .name = "Task04",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task05 */
osThreadId_t Task05Handle;
const osThreadAttr_t Task05_attributes = {
  .name = "Task05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task06 */
osThreadId_t Task06Handle;
const osThreadAttr_t Task06_attributes = {
  .name = "Task06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task07 */
osThreadId_t Task07Handle;
const osThreadAttr_t Task07_attributes = {
  .name = "Task07",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task08 */
osThreadId_t Task08Handle;
const osThreadAttr_t Task08_attributes = {
  .name = "Task08",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task09 */
osThreadId_t Task09Handle;
const osThreadAttr_t Task09_attributes = {
  .name = "Task09",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task10 */
osThreadId_t Task10Handle;
const osThreadAttr_t Task10_attributes = {
  .name = "Task10",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StartTimerAndCl */
osThreadId_t StartTimerAndClHandle;
const osThreadAttr_t StartTimerAndCl_attributes = {
  .name = "StartTimerAndCl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for startHwTimerBalansir */
osTimerId_t startHwTimerBalansirHandle;
const osTimerAttr_t startHwTimerBalansir_attributes = {
  .name = "startHwTimerBalansir"
};
/* USER CODE BEGIN PV */
//osThreadId_t Task11Handle;
//const osThreadAttr_t Task11_attributes = { .name = "Task11", .stack_size = 128
//		* 4, .priority = (osPriority_t) osPriorityLow, };
=======
 UART_HandleTypeDef huart2;
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
<<<<<<< HEAD
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);
void StartTask09(void *argument);
void StartTask10(void *argument);
void StartTask11(void *argument);
void startHwTimerBalansir01(void *argument);
=======
void StartDefaultTask(void const * argument);
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

<<<<<<< HEAD
static char statusPrintMem = 1;

PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}


void configureTimerForRunTimeStats(void)
{
//	HAL_TIM_Base_Start(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);

}

unsigned long getRunTimeCounterValue(void)
{
	size_t ret = ( __HAL_TIM_GET_COUNTER(&htim2) );
		__HAL_TIM_SET_COUNTER(&htim2,0);
	return (ret);
}






uint8_t bRxData[2];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	statusPrintMem = bRxData[0] - '0';
	HAL_UART_Receive_IT(huart, (uint8_t*) (bRxData), 1);
}

#define inlineWhile10k() {volatile short i=100000;	while(i)i--;}





#define MAX_TASKS_MONITOR (20)
TaskStatus_t taskState[MAX_TASKS_MONITOR];

//xTimerHandle xTimer
void balansirTask50(){

	#define fromMX(xx) xx##_GPIO_Port, xx##_Pin
	HAL_GPIO_TogglePin(fromMX(BLINK));

	const TickType_t x100ms = pdMS_TO_TICKS(100); //Convert to tick time for delay
	UBaseType_t task_count;

//	while (1)
	{
		char statusPrint2=statusPrintMem;

		if (statusPrint2 > 0) {
			printf("\033[2J");    // ESC seq for clear entire screen
			printf("\033[H"); // ESC seq for move cursor at left-top corner
			printf(
					"List Tasks freeRTOS (0-off, 1-on, 2-single print\n\n\r");
		}

		task_count = uxTaskGetNumberOfTasks();

		if ( task_count <= MAX_TASKS_MONITOR ) {
			unsigned long _total_runtime;

			task_count = uxTaskGetSystemState(taskState,MAX_TASKS_MONITOR, &_total_runtime);
			//подсчет общего времени
			size_t i=0,summTime=1,c=0;

			while( i < task_count ){
				summTime+=taskState[i].ulRunTimeCounter;
				i++;
			}




			if (statusPrint2 > 0) {
				printf("State\tPrior\tStack\tTime\t Prc \tUnicN\tTaskName\n\r");
				for (int task = 0; task < task_count; task++) {
					char _task_state_to_char(eTaskState state) {
						switch (state) {
						case eReady:
							return 'R';
						case eBlocked:
							return 'B';
						case eSuspended:
							return 'S';
						case eDeleted:
							return 'D';
						default:
							return '-';
						}
					}
					if (statusPrint2 > 0) {
						size_t i=0;
						while( i < MAX_TASKS_MONITOR ){
							if (taskState[i].xTaskNumber == (task+1)){
								break;
							}
							i++;
						}

						size_t ttime=(1000*taskState[i].ulRunTimeCounter/summTime);

						printf("[%c]"  "\t%u"  "\t%u"  "\t%u"   "\t%u.%u"  "\t%u"  "\t'%s'"  "\n\r",
								_task_state_to_char(taskState[i].eCurrentState)
								,taskState[i].uxCurrentPriority
								,taskState[i].usStackHighWaterMark
								,taskState[i].ulRunTimeCounter
								,ttime/10,ttime%10
								,taskState[i].xTaskNumber
								,taskState[i].pcTaskName

						);
					}
				}
			}//whilePrint
			if (statusPrint2 > 0) {
				printf("Current Heap Free Size: %u\n\r",
						xPortGetFreeHeapSize());

				//			    printf("Minimal Heap Free Size: %u",
				//			                          xPortGetMinimumEverFreeHeapSize());

				printf("Total RunTime:  %u ms\n\r", _total_runtime);

				//			    printf("System Uptime:  %u ms\r\n",
				//							      xTaskGetTickCount() * portTICK_PERIOD_MS);
			}
		}else{
			printf("много тасков и не влазиют в дамп буфер");
		}
		if (statusPrint2 > 1){
			statusPrintMem = 0 ;
		}

	//поиск 2й задачи и
		size_t i=0,summ=0,c=0;

		while( i < task_count ){
			size_t tn = taskState[i].xTaskNumber;
			if (tn == 2){
				c=i;
			}else //складываем время всех, кроме 2й задачи!
			//if ((tn>=3) & (tn<=11))
			{
				summ+=taskState[i].ulRunTimeCounter;
			}
			i++;
		}

		if ( summ > taskState[c].ulRunTimeCounter ){
			//отдаем часть таску02
			osThreadSetPriority(Task03Handle,osPriorityNormal-1);
			osThreadSetPriority(Task04Handle,osPriorityNormal-1);
			osThreadSetPriority(Task05Handle,osPriorityNormal-1);
			osThreadSetPriority(Task06Handle,osPriorityNormal-1);
			osThreadSetPriority(Task07Handle,osPriorityNormal-1);
			osThreadSetPriority(Task08Handle,osPriorityNormal-1);
			osThreadSetPriority(Task09Handle,osPriorityNormal-1);
			osThreadSetPriority(Task10Handle,osPriorityNormal-1);
			osThreadSetPriority(Task11Handle,osPriorityNormal-1);
			osThreadSetPriority(defaultTaskHandle,osPriorityNormal-1);
		}else{
			osThreadSetPriority(defaultTaskHandle,osPriorityNormal);
			osThreadSetPriority(Task03Handle,osPriorityNormal);
			osThreadSetPriority(Task04Handle,osPriorityNormal);
			osThreadSetPriority(Task05Handle,osPriorityNormal);
			osThreadSetPriority(Task06Handle,osPriorityNormal);
			osThreadSetPriority(Task07Handle,osPriorityNormal);
			osThreadSetPriority(Task08Handle,osPriorityNormal);
			osThreadSetPriority(Task09Handle,osPriorityNormal);
			osThreadSetPriority(Task10Handle,osPriorityNormal);
			osThreadSetPriority(Task11Handle,osPriorityNormal);
		}

//		TickType_t xLastWakeTime;  // Define a variable to hold wakeuptime
//		xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.
//		vTaskDelayUntil(&xLastWakeTime, 5*x100ms);   // Wait 100ms
//

	}
}
=======
#define ECHO_SEC (2)
#define ECHODELAY_MS ((uint32_t)ECHO_SEC*1000)
#define NBUFFECHO (9600/10*ECHO_SEC)

#define NQUEUE 	(9600/10*10/1000+1)

volatile uint8_t rxEchoISR[1+1];//в него Rx драйвер IRQ
volatile uint8_t bTxEchoISR[1+1];//буфер передачи

typedef struct{
	uint32_t time;
	uint8_t data;
}data_t;

data_t RxData[NBUFFECHO];
data_t RxQ[NQUEUE];	 //буфер для переноса из очереди в рабочий RxData[]

static size_t timeTail=0;
static size_t RxEchoTail=0;
static size_t nTxDataQ=0;
static size_t TxIndex=0;

volatile uint8_t EchoToTransfer=0;

xQueueHandle rxQueueEchoData;

#define phUartEcho &huart2

#define ECHOUART USART2


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t TxXferCount=huart->TxXferCount;
	uint16_t TxXferSize=huart->TxXferSize;

	if (huart-> Instance == ECHOUART) // Echo
	{
		if(TxXferSize == 0 ) // какая-то ошибка
		{
			HAL_UART_Transmit_IT(huart, (uint8_t *)(bTxEchoISR),EchoToTransfer);
		}
		else if (TxXferSize > 0 & TxXferCount < TxXferSize)
		{
			if(TxXferCount>0)
				HAL_UART_Transmit_IT(huart, (uint8_t *)(bTxEchoISR+TxXferSize-TxXferCount),TxXferCount);
		}
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart-> Instance == ECHOUART)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		if(RxXferSize == 0)
		{
			printf("HAL_UART_RxCpltCallback Echo????");
		}
		else if(RxXferSize > 0 && RxXferCount < RxXferSize)
		{
			BaseType_t xHigherPriorityTaskWoken;
			xHigherPriorityTaskWoken = pdFALSE;

			for(int ii=0; ii<  (RxXferSize-RxXferCount); ii++)
			{
				data_t tmp;
					tmp.time = HAL_GetTick(); //захват времени, по соотношению к 2 сек, шкала времени избыточна
					tmp.data = rxEchoISR[ii];
				xQueueSendToBackFromISR(rxQueueEchoData, &tmp , &xHigherPriorityTaskWoken );
			}


			if( xHigherPriorityTaskWoken )
			{
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}

		}
		HAL_UART_Receive_IT(huart, (uint8_t *)(rxEchoISR),1);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart-> Instance == ECHOUART)
	{
		uint16_t RxXferCount=huart->RxXferCount;
		uint16_t RxXferSize=huart->RxXferSize;

		printf("< HAL_UART_ErrorCallback ECHOUART\n");
		printf("< RxXferCount=%d RxXferSize=%d\n",RxXferCount,RxXferSize);

		HAL_UART_Receive_IT(phUartEcho, (uint8_t *)(rxEchoISR),1);
	}

}
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
	printf("HAL_UART_AbortReceiveCpltCallback \n");
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
{
	printf("HAL_UART_AbortTransmitCpltCallback \n");
}
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
	printf("HAL_UART_AbortCpltCallback \n");
}






>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092





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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
<<<<<<< HEAD


	printf("MCU started ...\n\r");

	HAL_UART_Receive_IT(&huart2, (uint8_t*) (bRxData), 1);

	//	HAL_TIM_Base_Start_IT(&htim7); //timer balansir

  HAL_TIM_Base_Start(&htim2);//timer подсчета rtos

=======
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
<<<<<<< HEAD
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
=======
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of startHwTimerBalansir */
  startHwTimerBalansirHandle = osTimerNew(startHwTimerBalansir01, osTimerPeriodic, NULL, &startHwTimerBalansir_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
<<<<<<< HEAD
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task01 */
  Task01Handle = osThreadNew(StartDefaultTask, NULL, &Task01_attributes);

  /* creation of Task02 */
  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

  /* creation of Task03 */
  Task03Handle = osThreadNew(StartTask03, NULL, &Task03_attributes);

  /* creation of Task04 */
  Task04Handle = osThreadNew(StartTask04, NULL, &Task04_attributes);

  /* creation of Task05 */
  Task05Handle = osThreadNew(StartTask05, NULL, &Task05_attributes);

  /* creation of Task06 */
  Task06Handle = osThreadNew(StartTask06, NULL, &Task06_attributes);

  /* creation of Task07 */
  Task07Handle = osThreadNew(StartTask07, NULL, &Task07_attributes);

  /* creation of Task08 */
  Task08Handle = osThreadNew(StartTask08, NULL, &Task08_attributes);

  /* creation of Task09 */
  Task09Handle = osThreadNew(StartTask09, NULL, &Task09_attributes);

  /* creation of Task10 */
  Task10Handle = osThreadNew(StartTask10, NULL, &Task10_attributes);
=======
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092

  /* creation of StartTimerAndCl */
  StartTimerAndClHandle = osThreadNew(StartTask11, NULL, &StartTimerAndCl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
<<<<<<< HEAD
  //Task11Handle = osThreadNew(StartTask11, NULL, &Task11_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */



  /* USER CODE END RTOS_EVENTS */

=======
  /* USER CODE END RTOS_THREADS */

>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
<<<<<<< HEAD
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
=======
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLINK_GPIO_Port, BLINK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLINK_Pin */
  GPIO_InitStruct.Pin = BLINK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLINK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
<<<<<<< HEAD

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the Task50prcnt thread.
 * @param  argument: Not used
 * @retval None
 */
=======
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
<<<<<<< HEAD

//	 if (osTimerStart(BallansirHandle,200) != osOK){
//		  printf("timer balansir not run");
//	  }

	/* Infinite loop */
	for (;;) {
		inlineWhile10k();
		//balansirTask50();
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the Task02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the Task03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Function implementing the Task04 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Function implementing the Task05 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
 * @brief Function implementing the Task06 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
 * @brief Function implementing the Task07 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
 * @brief Function implementing the Task08 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask08 */
}

/* USER CODE BEGIN Header_StartTask09 */
/**
 * @brief Function implementing the Task09 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask09 */
void StartTask09(void *argument)
{
  /* USER CODE BEGIN StartTask09 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
  /* USER CODE END StartTask09 */
}

/* USER CODE BEGIN Header_StartTask10 */
/**
 * @brief Function implementing the Task10 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask10 */
void StartTask10(void *argument)
{
  /* USER CODE BEGIN StartTask10 */
	/* Infinite loop */
	for (;;) {
		inlineWhile10k();

	}
	//osTimerStart(startHwTimerBalansirHandle, 200);
  /* USER CODE END StartTask10 */
}

/* USER CODE BEGIN Header_StartTask11 */
/**
* @brief Function implementing the Task11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask11 */
void StartTask11(void *argument)
{
  /* USER CODE BEGIN StartTask11 */


	osTimerStart(startHwTimerBalansirHandle, 200);

  /* Infinite loop */
  for(;;)
  {
	inlineWhile10k();
  }
  /* USER CODE END StartTask11 */
}

/* startHwTimerBalansir01 function */
void startHwTimerBalansir01(void *argument)
{
  /* USER CODE BEGIN startHwTimerBalansir01 */
	balansirTask50();
  /* USER CODE END startHwTimerBalansir01 */
=======

	  /* Infinite loop */
	  for(;;)
	  {

			HAL_StatusTypeDef res;

			rxQueueEchoData = xQueueCreate( NQUEUE , sizeof( data_t ) );

			if( rxQueueEchoData == NULL )
				printf("rxQueueEcho Queue was not created and must not be used.\n");

			// Обязательно сначала запускаем прием
			if(res=HAL_UART_Receive_IT(phUartEcho, (uint8_t *)rxEchoISR, 1) == HAL_OK)
				; //printf("HAL_OK\n");
			else if(res = HAL_ERROR)
				printf("Echo HAL_ERROR\n");
			else  if(res = HAL_BUSY)
				printf("Echo HAL_BUSY\n");


			const TickType_t xMaxExpectedBlockTime = pdMS_TO_TICKS( 1 );

			for(;;)
			{
				uint8_t numbsData = xQueueReceive(rxQueueEchoData, (data_t*)&RxQ,  xMaxExpectedBlockTime);
				if(numbsData > 0){
					nTxDataQ+=numbsData;
					if ( nTxDataQ >= NBUFFECHO ){
						printf("чего-то пошло не то, данные не ушли!");
					}
					for(int ii=0; ii< numbsData ; ii++)
					{
						RxData[timeTail].time=RxQ[ii].time;
						RxData[timeTail].data=RxQ[ii].data;
						if ( ++timeTail == NBUFFECHO ){
							timeTail = 0 ;
						}
					}

				}//забор из очередей
				//
				if (nTxDataQ){ //имеются данные для передачи
					volatile uint32_t time=HAL_GetTick();
					if (  ( time-RxData[TxIndex].time ) >= ECHODELAY_MS  ){


						//просто проверка Tx буфера
						if ( phUartEcho.TxXferCount > 0 ){
							vTaskDelay(1);//и опустошение, если биток, не блокируя, но можно while+timeout
						}

						if (HAL_UART_Transmit_IT(phUartEcho, (uint8_t *)&RxData[TxIndex].data,1 ) != HAL_OK);

						TxIndex++;
						if (TxIndex == NBUFFECHO){
							TxIndex = 0 ;
						}
						nTxDataQ--;
						if( nTxDataQ > NBUFFECHO ){
							printf("чего-то не то, nTxDataQ<0!");
						}
					}
				}
					#define fromMX(xx) xx##_GPIO_Port, xx##_Pin
					static char c;
				vTaskDelay( pdMS_TO_TICKS(1) );
					if ((++c & 63) == 63){
						HAL_GPIO_TogglePin(fromMX(BLINK));//работает! 63мс+блок очереди
					}
			}
	  }




  /* USER CODE END 5 */
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
<<<<<<< HEAD
//	 if (htim->Instance == TIM7) {
//		 balansirTask50();
//	 }

=======
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
<<<<<<< HEAD
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
=======
>>>>>>> bffff311b8796d2470fecb2dbb929414d63ed092
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
