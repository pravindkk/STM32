/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../PeripheralDrivers/Inc/oled.h"
#include "../../PeripheralDrivers/Inc/ICM20948.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include "../../PeripheralDrivers/Src/oled.c"
#include "../../PeripheralDrivers/Src/ICM20948.c"


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
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for commandTask */
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ADCTask */
osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "ADCTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for moveDistTask */
osThreadId_t moveDistTaskHandle;
const osThreadAttr_t moveDistTask_attributes = {
  .name = "moveDistTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FLTask */
osThreadId_t FLTaskHandle;
const osThreadAttr_t FLTask_attributes = {
  .name = "FLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FRTask */
osThreadId_t FRTaskHandle;
const osThreadAttr_t FRTask_attributes = {
  .name = "FRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BLTask */
osThreadId_t BLTaskHandle;
const osThreadAttr_t BLTask_attributes = {
  .name = "BLTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BRTask */
osThreadId_t BRTaskHandle;
const osThreadAttr_t BRTask_attributes = {
  .name = "BRTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for batteryTask */
osThreadId_t batteryTaskHandle;
const osThreadAttr_t batteryTask_attributes = {
  .name = "batteryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for moveDistObsTask */
osThreadId_t moveDistObsTaskHandle;
const osThreadAttr_t moveDistObsTask_attributes = {
  .name = "moveDistObsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FPFirstObsTurnL */
osThreadId_t FPFirstObsTurnLHandle;
const osThreadAttr_t FPFirstObsTurnL_attributes = {
  .name = "FPFirstObsTurnL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FPFirstObsTurnR */
osThreadId_t FPFirstObsTurnRHandle;
const osThreadAttr_t FPFirstObsTurnR_attributes = {
  .name = "FPFirstObsTurnR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FPScndObsTurnL */
osThreadId_t FPScndObsTurnLHandle;
const osThreadAttr_t FPScndObsTurnL_attributes = {
  .name = "FPScndObsTurnL",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FPScndObsTurnR */
osThreadId_t FPScndObsTurnRHandle;
const osThreadAttr_t FPScndObsTurnR_attributes = {
  .name = "FPScndObsTurnR",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t RX_BUFFER_SIZE = 5;
uint8_t aRxBuffer[10];

typedef struct _command {
	uint8_t index;
	uint16_t val;
} Command;

uint8_t CMD_BUFFER_SIZE = 12;
typedef struct _commandQueue {
	uint8_t head;
	uint8_t tail;
	uint8_t size;
	Command buffer[12];
} CommandQueue;

CommandQueue cQueue;

Command curCmd;
uint8_t rxMsg[16];
char ch[25];
char US_dist[16];
char vert_dist[16];
char horiz_dist[16];


uint8_t manualMode = 0;

// PID
float targetAngle = 0;
float angleNow = 0;
float tmpAngle = 0;
uint8_t readGyroZData[2];
int16_t gyroZ;
int16_t previousGyroZ;
uint16_t newDutyL, newDutyR;
uint32_t last_curTask_tick = 0;

float targetDist = 0;
uint16_t curDistTick = 0;
uint16_t targetDistTick = 0;
uint16_t dist_dL = 0;
uint16_t lastDistTick_L = 0;


float angleNowPrint = 0;
uint8_t readGyroZDataPrint[2];
int16_t gyroZPrint;
int16_t previousGyroZPrint;

typedef struct _pidConfig {
	float Kp;
	float Ki;
	float Kd;
	float ek1;
	float ekSum;
} PIDConfig;

PIDConfig pidSlow, pidTSlow, pidFast;
PIDConfig pidServo;

typedef struct _commandConfig {
	uint16_t leftDuty;
	uint16_t rightDuty;
	float servoTurnVal;
	float targetAngle;
	uint8_t direction;
} CmdConfig;

#define CONFIG_FL00 7
#define CONFIG_FR00 8
#define CONFIG_BL00 9
#define CONFIG_BR00 10

#define CONFIG_FL20 11
#define CONFIG_FR20 12
#define CONFIG_BL20 13
#define CONFIG_BR20 14

#define CONFIG_FL30 15
#define CONFIG_FR30 16
#define CONFIG_BL30 17
#define CONFIG_BR30 18

#define CONFIG_FP_FL90 19
#define CONFIG_FP_FR90 20
#define CONFIG_FP_BL90 21
#define CONFIG_FP_BR90 22
#define CONFIG_FP_FL180 23
#define CONFIG_FP_FR180 24


// Radius
CmdConfig cfgs[25] = {
	{0,0,SERVO_CENTER,0, DIR_FORWARD}, // STOP
	{600, 600, SERVO_CENTER, 0, DIR_FORWARD}, // FW00
	{600, 600, SERVO_CENTER, 0, DIR_BACKWARD}, // BW00

	{800, 1200, 30, 0, DIR_FORWARD}, // FL--
	{1200, 800, 115, 0, DIR_FORWARD}, // FR--
	{800, 1200, 30, 0, DIR_BACKWARD}, // BL--
	{1200, 800, 115, 0, DIR_BACKWARD}, // BR--

	{700, 2000, 30, 90.6, DIR_FORWARD}, // FL00
	{2000, 300, 115 ,-87.8, DIR_FORWARD}, // FR00
	{600, 2000, 30, -88.6, DIR_BACKWARD}, // BL00
	{2500, 300, 115, 89.7, DIR_BACKWARD}, // BR00,

	{850, 1800, 30, 88.2, DIR_FORWARD}, // FL20
	{1800, 300, 115 ,-87.7, DIR_FORWARD}, // FR20
	{500, 1800, 30, -86.8, DIR_BACKWARD}, // BL20
	{1800, 400, 115, 90.4, DIR_BACKWARD}, // BR20,

	{300, 4500, 30, 89.8, DIR_FORWARD}, // FL30
	{1500, 1500, 108, -86.5, DIR_FORWARD}, // FR30
	{1500, 1500, 30, -87, DIR_BACKWARD}, // BL30
	{1500, 1100, 115, 88, DIR_BACKWARD}, // BR30
    
    {500, 3200, 24, 85.5, DIR_FORWARD}, // FP_FL_90
    {3200, 500, 120, -86, DIR_FORWARD}, // FP_FR_90
    {2500, 2500, 30, -87.5, DIR_BACKWARD}, // FP_BL_90
    {2500, 2100, 115, 88, DIR_BACKWARD}, // FP_BR_90
    {500, 3200, 30, 177.5, DIR_FORWARD}, // FP_FL_180
    {3200, 500, 115, -176, DIR_FORWARD}, // FP_FR_180
};

enum TASK_TYPE{
	TASK_MOVE,
	TASK_MOVE_BACKWARD,
	TASK_FL,
	TASK_FR,
	TASK_BL,
	TASK_BR,
	TASK_ADC,
	TASK_MOVE_OBS,
	TASK_BUZZER,
	TASK_FP_XL,
	TASK_FP_XR,
	TASK_FP_YL,
	TASK_FP_YR,
	TASK_NONE
};
enum TASK_TYPE curTask = TASK_NONE, prevTask = TASK_NONE;

enum MOVE_MODE {
	SLOW,
	FAST
};
 enum MOVE_MODE moveMode = FAST;


uint16_t obsTick_IR = 0;

float obsDist_IR = 0, obsDist_US = 0;
//float IR_data_raw_acc = 0, dataPoint = 0;
uint16_t dataPoint = 0; uint32_t IR_data_raw_acc = 0;
float speedScale = 1;

float batteryVal;

// fastest path variable
float obs_a, x, angle_left, angle_right;

// fastest path variables --my version
float horizontal_dist_bef_turn = 0, vertical_distance_to_second_obs = 0;
float first_obs_first_stop_dist = 0;
float second_obs_dist = 0;
int RPI_ACK_COUNT = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
void runOledTask(void *argument);
void runCmdTask(void *argument);
void runADCTask(void *argument);
void runMoveDistTask(void *argument);
void runBuzzerTask(void *argument);
void runFLTask(void *argument);
void runFRTask(void *argument);
void runBLTask(void *argument);
void runBRTask(void *argument);
void runBatteryTask(void *argument);
void runMoveDistObsTask(void *argument);
void runFPFirstObsTurnLeftTask(void *argument);
void runFPFirstObsTurnRightTask(void *argument);
void runFPSecondObsTurnLeftTask(void *argument);
void runFPSecondObsTurnRightTask(void *argument);

/* USER CODE BEGIN PFP */
void PIDConfigInit(PIDConfig * cfg, const float Kp, const float Ki, const float Kd);
void PIDConfigReset(PIDConfig * cfg);

void StraightLineMove(const uint8_t speedMode);
void StraightLineMoveSpeedScale(const uint8_t speedMode, float * speedScale);
void RobotMoveDist(float * targetDist, const uint8_t dir, const uint8_t speedMode);
void RobotMoveDistObstacle(float * targetDist, const uint8_t speedMode);

void RobotTurn(float * targetAngle, CmdConfig cfgs);
void RobotTurnFastest(float * targetAngle);

uint32_t IC_Val1 = 0, IC_Val2 = 0;
uint8_t Is_First_Captured = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if the interrupt source is channel2 (TRI: TIM4_CH2)
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2); // read the first value
			Is_First_Captured = 1;  // set the first captured as true

			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			obsDist_US = (IC_Val2 > IC_Val1 ? (IC_Val2 - IC_Val1) : (65535 - IC_Val1 + IC_Val2)) * 0.034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
//			  snprintf(US_dist, sizeof(US_dist), "dist: %3.2lf", (float) obsDist_US);
//			  OLED_ShowString(0, 50, (char *) US_dist);
//			  OLED_Refresh_Gram();
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
//			HAL_TIM_Base_Stop_IT(&htim4);
			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC2);

//			if (recordAngleOnOvershoot) { // for fastest path challenge
//
//			}
//			if (abs(targetDist - obsDist_US) < 0.01) break;
//		  __SET_MOTOR_DIRECTION(obsDist_US >= targetDist);
//		  if (HAL_GetTick() - last_curTask_tick >=10) {
//	//		  speedScale = 1;
//			  speedScale = abs(obsDist_US - targetDist) / 15; // slow down at 15cm
//			  speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
//			  StraightLineMoveSpeedScale(SPEED_MODE_2, &speedScale);
//
//			  last_curTask_tick = HAL_GetTick();
//		  }
		}
	}
}
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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  ICM20948_init(&hi2c1,0,GYRO_FULL_SCALE_2000DPS);

  // initialise command queue
  curCmd.index = 100;
  curCmd.val = 0;

  cQueue.head = 0;
  cQueue.tail = 0;
  cQueue.size = CMD_BUFFER_SIZE;
  for (int i = 0; i < CMD_BUFFER_SIZE;i++) {
	  Command cmd;
	  cmd.index = 100;
	  cmd.val = 0;
	  cQueue.buffer[i] = cmd;
  }

  PIDConfigInit(&pidTSlow, 2.5, 0.0,0.8);
  PIDConfigInit(&pidSlow, 3.8, 0.0,0);
  PIDConfigInit(&pidFast, 1.5, 0.0,0);
  PIDConfigInit(&pidServo, 3.8, 0.0,0);

//  PIDConfigInit(&pidFast, 0.75, 0.0,0);

  	HAL_UART_Receive_IT(&huart3, aRxBuffer,RX_BUFFER_SIZE);

	// servo motor turn
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	// motor backwheel move
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	// encoder monitor speed and distance
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

	//adjust steering
	__RESET_SERVO_TURN(&htim1);


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

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(runOledTask, NULL, &oledTask_attributes);

  /* creation of commandTask */
  commandTaskHandle = osThreadNew(runCmdTask, NULL, &commandTask_attributes);

  /* creation of ADCTask */
  ADCTaskHandle = osThreadNew(runADCTask, NULL, &ADCTask_attributes);

  /* creation of moveDistTask */
  moveDistTaskHandle = osThreadNew(runMoveDistTask, NULL, &moveDistTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(runBuzzerTask, NULL, &buzzerTask_attributes);

  /* creation of FLTask */
  FLTaskHandle = osThreadNew(runFLTask, NULL, &FLTask_attributes);

  /* creation of FRTask */
  FRTaskHandle = osThreadNew(runFRTask, NULL, &FRTask_attributes);

  /* creation of BLTask */
  BLTaskHandle = osThreadNew(runBLTask, NULL, &BLTask_attributes);

  /* creation of BRTask */
  BRTaskHandle = osThreadNew(runBRTask, NULL, &BRTask_attributes);

  /* creation of batteryTask */
  batteryTaskHandle = osThreadNew(runBatteryTask, NULL, &batteryTask_attributes);

  /* creation of moveDistObsTask */
  moveDistObsTaskHandle = osThreadNew(runMoveDistObsTask, NULL, &moveDistObsTask_attributes);

  /* creation of FPFirstObsTurnL */
  FPFirstObsTurnLHandle = osThreadNew(runFPFirstObsTurnLeftTask, NULL, &FPFirstObsTurnL_attributes);

  /* creation of FPFirstObsTurnR */
  FPFirstObsTurnRHandle = osThreadNew(runFPFirstObsTurnRightTask, NULL, &FPFirstObsTurnR_attributes);

  /* creation of FPScndObsTurnL */
  FPScndObsTurnLHandle = osThreadNew(runFPSecondObsTurnLeftTask, NULL, &FPScndObsTurnL_attributes);

  /* creation of FPScndObsTurnR */
  FPScndObsTurnRHandle = osThreadNew(runFPSecondObsTurnRightTask, NULL, &FPScndObsTurnR_attributes);

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
  sConfig.Channel = ADC_CHANNEL_11;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim1.Init.Prescaler = 320;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
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
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|TRI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin TRI_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|TRI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// HAL_UART_RxCpltCallback evoked when buffer is full
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
	// prevent unused argument(s) compilation warning
	UNUSED(huart);
	int val;

	val = (aRxBuffer[2] - 48) * 10 + (aRxBuffer[3] - 48);
	if (aRxBuffer[1] >= '0' && aRxBuffer[1] <= '9') val += (aRxBuffer[1] - 48) * 100;

	manualMode = 0;

	if (aRxBuffer[0] == 'S' && aRxBuffer[1] == 'T') { // only STOP can preempt any greedy task
//		__ADD_COMMAND(cQueue, 0, 0); // stop
		__ON_TASK_END(&htim8, prevTask, curTask);
		  angleNow = 0; gyroZ = 0; // reset angle for PID
		PIDConfigReset(&pidTSlow);
		PIDConfigReset(&pidSlow);
		PIDConfigReset(&pidFast);
		curDistTick = 0;
		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
		}
		else {
			__READ_COMMAND(cQueue, curCmd, rxMsg);
		}
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'I') { // only FIN can preempt any greedy task
	//		__ADD_COMMAND(cQueue, 0, 0); // stop
		__ACK_TASK_DONE(&huart3, rxMsg);
	}
	else if (aRxBuffer[0] == 'F' && (aRxBuffer[1] == 'W' || aRxBuffer[1] == 'S')) { //FW or FS
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		moveMode = aRxBuffer[1] == 'S' ? SLOW : FAST;
		__ADD_COMMAND(cQueue, 1, val);
	}
	else if (aRxBuffer[0] == 'B' && (aRxBuffer[1] == 'W' || aRxBuffer[1] == 'S')) { //BW or BS
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		moveMode = aRxBuffer[1] == 'S' ? SLOW : FAST;
		__ADD_COMMAND(cQueue, 2, val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'L') { // FL
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 3 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'F' && aRxBuffer[1] == 'R') { // FR
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 4 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'L') { // BL
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 5 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'B' && aRxBuffer[1] == 'R') { // BR
		manualMode = aRxBuffer[2] == '-' && aRxBuffer[3] == '-';
		__ADD_COMMAND(cQueue, 6 + (manualMode ? 0 : 4), val);
	}
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 11, val); // TL turn left max
	else if (aRxBuffer[0] == 'T' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 12, val); // TR turn right max
	else if (aRxBuffer[0] == 'I' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 13, val); // test IR sensor
	else if (aRxBuffer[0] == 'D' && aRxBuffer[1] == 'T') __ADD_COMMAND(cQueue, 14, val); // DT move until specified distance from obstacle
	else if (aRxBuffer[0] == 'X' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 18, val); // Fastest Path FIRST obstacle turn LEFT
	else if (aRxBuffer[0] == 'X' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 19, val); // Fastest Path FIRST obstacle turn RIGHT
	else if (aRxBuffer[0] == 'Y' && aRxBuffer[1] == 'L') __ADD_COMMAND(cQueue, 20, val); // Fastest Path SECOND obstacle turn LEFT
	else if (aRxBuffer[0] == 'Y' && aRxBuffer[1] == 'R') __ADD_COMMAND(cQueue, 21, val); // Fastest Path SECOND obstacle turn RIGHT
	else if (aRxBuffer[0] == 'Z' && aRxBuffer[1] == 'Z') __ADD_COMMAND(cQueue, 15, val); // ZZ buzzer
	else if (aRxBuffer[0] == 'W' && aRxBuffer[1] == 'X') __ADD_COMMAND(cQueue, 16, val); // WX fastest path
	else if (aRxBuffer[0] == 'W' && aRxBuffer[1] == 'N') __ADD_COMMAND(cQueue, 17, val); // WN fastest path v2
	else if (aRxBuffer[0] == 'A') __ADD_COMMAND(cQueue, 88, val); // anti-clockwise rotation with variable
	else if (aRxBuffer[0] == 'C') __ADD_COMMAND(cQueue, 89, val); // clockwise rotation with variable



	if (!__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
		__READ_COMMAND(cQueue, curCmd, rxMsg);
	}

	// clear aRx buffer
	  __HAL_UART_FLUSH_DRREGISTER(&huart3);
	  HAL_UART_Receive_IT(&huart3, aRxBuffer, RX_BUFFER_SIZE);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) {
//	 UNUSED(huart);
//
//	 UART_Tx_Done = 1;
//}

int clickOnce = 0;
int targetD = 5;
uint8_t tempDir = 1 ;
int8_t step = 0;
uint8_t turnMode = 2;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (clickOnce) return;
	if (GPIO_Pin == SW1_Pin) {
		clickOnce = 1;


//		manualMode = 1;
//		moveMode = FAST;
//		__ADD_COMMAND(cQueue, 1, 90);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);

		__ADD_COMMAND(cQueue, 7 + step, 0);
		__READ_COMMAND(cQueue, curCmd, rxMsg);

		step = (step + 1) % 4;
//		step = (step + 1) % 7;
//		__ADD_COMMAND(cQueue, 17, turnMode + 1);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);
//		turnMode = (turnMode + 1) % 4;
//		__ADD_COMMAND(cQueue, 1, 100);
//		__READ_COMMAND(cQueue, curCmd, rxMsg);
	}

}

void PIDConfigInit(PIDConfig * cfg, const float Kp, const float Ki, const float Kd) {
	cfg->Kp = Kp;
	cfg->Ki = Ki;
	cfg->Kd = Kd;
	cfg->ek1 = 0;
	cfg->ekSum = 0;
}

void PIDConfigReset(PIDConfig * cfg) {
	cfg->ek1 = 0;
	cfg->ekSum = 0;
}


int8_t dir = 1;
int correction = 0;
//PIDConfig curPIDConfig;

void StraightLineMove(const uint8_t speedMode) {
	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ); // polling
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction
	angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;

	if (speedMode == SPEED_MODE_T) __PID_SPEED_T(pidTSlow, angleNow, correction, dir, newDutyL, newDutyR);
	else if (speedMode == SPEED_MODE_2) __PID_SPEED_2(pidFast, angleNow, correction, dir, newDutyL, newDutyR);
	else if (speedMode == SPEED_MODE_1) __PID_SPEED_1(pidSlow, angleNow, correction, dir, newDutyL, newDutyR);

	__SET_MOTOR_DUTY(&htim8, newDutyL, newDutyR);
}

void StraightLineMoveSpeedScale(const uint8_t speedMode, float * speedScale) {
	__Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ); // polling
	dir = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2) ? 1 : -1; // use only one of the wheel to determine car direction
	angleNow += ((gyroZ >= -4 && gyroZ <= 11) ? 0 : gyroZ * 0.05); // / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
	if (speedMode == SPEED_MODE_1) __PID_SPEED_1(pidSlow, angleNow, correction, dir, newDutyL, newDutyR);
	else if (speedMode == SPEED_MODE_2) __PID_SPEED_2(pidFast, angleNow, correction, dir, newDutyL, newDutyR);

	__SET_MOTOR_DUTY(&htim8, newDutyL * (*speedScale), newDutyR * (*speedScale));
}

void RobotMoveDist(float * targetDist, const uint8_t dir, const uint8_t speedMode) {
	angleNow = 0; gyroZ = 0; // reset angle for PID
	PIDConfigReset(&pidTSlow);
	PIDConfigReset(&pidSlow);
	PIDConfigReset(&pidFast);
	curDistTick = 0;

	if (speedMode == SPEED_MODE_1) {
		__GET_TARGETTICK(*targetDist-1, targetDistTick);
	} else {
		__GET_TARGETTICK(*targetDist, targetDistTick);
	}
//	__GET_TARGETTICK(*targetDist, targetDistTick);

	last_curTask_tick = HAL_GetTick();
	__SET_MOTOR_DIRECTION(dir);
	__SET_ENCODER_LAST_TICK(&htim2, lastDistTick_L);
	do {
		__GET_ENCODER_TICK_DELTA(&htim2, lastDistTick_L, dist_dL);
		curDistTick += dist_dL;

		if (curDistTick >= targetDistTick) break;

		if (HAL_GetTick() - last_curTask_tick >= 10) {
			if (speedMode == SPEED_MODE_T) {
				StraightLineMove(SPEED_MODE_T);
			} else {
				speedScale = abs(curDistTick - targetDistTick) / 990; // start to slow down at last 990 ticks (15cm)
				if (speedMode == SPEED_MODE_1) speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale);
				else if (speedMode == SPEED_MODE_2)speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
				StraightLineMoveSpeedScale(speedMode, &speedScale);
			}

			last_curTask_tick = HAL_GetTick();
		}
	} while (1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
}

// RobotMoveDistObstacle must be called within a task(eg. runFastestPath) and not within an interrupt(eg. UART, EXTI)
// else osDelay won't work and TRI's timer interrupt can't be given chance to update obsDist_US
void RobotMoveDistObstacle(float * targetDist, const uint8_t speedMode) {
	angleNow = 0; gyroZ = 0;
	PIDConfigReset(&pidTSlow);
	PIDConfigReset(&pidSlow);
	PIDConfigReset(&pidFast);
	obsDist_US = 1000;
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	last_curTask_tick = HAL_GetTick();

	do {
	  HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	  __delay_us(&htim4, 10); // wait for 10us
	  HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
//	  HAL_TIM_Base_Start_IT(&htim4);
	  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
	  osDelay(10); // give timer interrupt chance to update obsDist_US value

	  if (fabs(obsDist_US - *targetDist) < 0.1) break;
	  __SET_MOTOR_DIRECTION( obsDist_US >= *targetDist - 0.1);
	  if (HAL_GetTick() - last_curTask_tick >=10) {
//		  speedScale = 1;
		  if (speedMode == SPEED_MODE_1) {
			  speedScale = abs(obsDist_US - *targetDist) / 15; // slow down at 15cm
			  speedScale = speedScale > 1 ? 1 : (speedScale < 0.75 ? 0.75 : speedScale);
			  StraightLineMoveSpeedScale(SPEED_MODE_1, &speedScale);
		  } else {
			  speedScale = abs(obsDist_US - *targetDist) / 15; // slow down at 15cm
			  speedScale = speedScale > 1 ? 1 : (speedScale < 0.4 ? 0.4 : speedScale);
			  StraightLineMoveSpeedScale(SPEED_MODE_2, &speedScale);
		  }


		  last_curTask_tick = HAL_GetTick();
	  }

	} while (1);

	__SET_MOTOR_DUTY(&htim8, 0, 0);
	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);
}

void RobotMoveDistObstacle_IR(float * targetDist) {
	dataPoint = 0; IR_data_raw_acc = 0; obsDist_IR = 1000;
	last_curTask_tick = HAL_GetTick();
//	__PEND_CURCMD(curCmd);

	do {
		__ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		if (*targetDist > 0 && abs(*targetDist - obsDist_IR) < 0.1) break;

		__SET_MOTOR_DIRECTION(obsDist_IR >= *targetDist);
	  if (HAL_GetTick() - last_curTask_tick >=10) {
//		  speedScale = 1;
		  speedScale = abs(obsDist_IR - *targetDist) / 15; // slow down at 15cm
		  speedScale = speedScale > 1 ? 1 : (speedScale < 0.3 ? 0.3 : speedScale);
		  StraightLineMoveSpeedScale(SPEED_MODE_2, &speedScale);

		  last_curTask_tick = HAL_GetTick();
	  }
//	  osDelay(5);
	} while (1);

//  __ON_TASK_END(&htim8, prevTask, curTask);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
	HAL_ADC_Stop(&hadc1);
}

void RobotTurn(float * targetAngle, CmdConfig cfgs) {
	angleNow = 0; gyroZ = 0;
	previousGyroZ = 0;
	float angle_overshot = 0;

	int servo_offset = 0;
	int new_servo_value = 0;
	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;  //
		  tmpAngle = angleNow;
//		  snprintf(ch, sizeof(ch), "angle:%-3.2lf", (float) angleNow);
//		  OLED_ShowString(0, 0, (char *) ch);
		  if (abs(angleNow - *targetAngle) < 20) {
			  float newLeftDuty = (float)(cfgs).leftDuty;
			  float newRightDuty = (float)(cfgs).rightDuty;
			  // Multiply by 0.5
			  newLeftDuty *= 0.7f;
			  newRightDuty *= 0.7f;
			  __SET_MOTOR_DUTY(&htim8, newLeftDuty, newRightDuty);
		  }
		  if (abs(angleNow - *targetAngle) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
//		  OLED_Refresh_Gram();
	  }
	} while(1);

	angle_overshot = angleNow - *targetAngle;
//	angle_overshot = angleNow - 100;
	if (abs(angle_overshot) > 0.1) {
		servo_offset = (angle_overshot / (*targetAngle)) * SERVO_RIGHT_MAX;

		if (cfgs.direction == DIR_FORWARD) {
			__SET_MOTOR_DIRECTION(DIR_BACKWARD);
		} else {
			__SET_MOTOR_DIRECTION(DIR_FORWARD);
		}

		if (*targetAngle > 0) {
			if (angleNow > *targetAngle) {
				new_servo_value = SERVO_CENTER - servo_offset;
			} else if (angleNow < *targetAngle) {
				new_servo_value = SERVO_CENTER + servo_offset;
			}
		} else if (*targetAngle < 0) {
			if (angleNow > *targetAngle) {
				new_servo_value = SERVO_CENTER - servo_offset;
			} else if (angleNow < *targetAngle) {
				new_servo_value = SERVO_CENTER + servo_offset;
			}
		}

		__SET_SERVO_TURN(&htim1, new_servo_value);


		do {
		  __SET_MOTOR_DUTY(&htim8, 1600, 1600);
		  last_curTask_tick = HAL_GetTick();
		} while(HAL_GetTick() - last_curTask_tick >= 20);

		__SET_MOTOR_DUTY(&htim8, 0, 0);
		__RESET_SERVO_TURN(&htim1);

		if (cfgs.direction == DIR_FORWARD) {
			__SET_MOTOR_DIRECTION(DIR_FORWARD);
		} else {
			__SET_MOTOR_DIRECTION(DIR_BACKWARD);
		}

		last_curTask_tick = HAL_GetTick();
		do {
		  __SET_MOTOR_DUTY(&htim8, 1600, 1600);
		  last_curTask_tick = HAL_GetTick();
		} while(HAL_GetTick() - last_curTask_tick >= 20);
	}



	__SET_MOTOR_DUTY(&htim8, 0, 0);
	__RESET_SERVO_TURN(&htim1);

	if ((*targetAngle > 0 && (cfgs).direction == DIR_FORWARD) ||
			(*targetAngle < 0 && (cfgs).direction == DIR_BACKWARD)) {
		__SET_SERVO_TURN_MAX(&htim1, 1);
		__RESET_SERVO_TURN(&htim1);
	}

}

void RobotTurn_V2(float * targetAngle) {
	angleNow = 0; gyroZ = 0;
	previousGyroZ = 0;
	float angle_overshot = 0;
	int servo_offset = 0;
	int new_servo_value = 0;
	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;  //
		  if (abs(angleNow - *targetAngle) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(1);
	angle_overshot = angleNow - *targetAngle;
	servo_offset = (angle_overshot / (*targetAngle)) * SERVO_RIGHT_MAX;

	osDelay(1);

	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 20) { // sample gyro every 10ms
		  __SET_MOTOR_DUTY(&htim8, 800, 800);
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(HAL_GetTick() - last_curTask_tick <= 20);

//	if (angle_overshot < 0.01)
//	{
//
//		new_servo_value = SERVO_RIGHT_MAX - servo_offset;
//		__SET_SERVO_TURN(&htim1, new_servo_value);
//		__SET_MOTOR_DIRECTION(DIR_BACKWARD);
//
//	}
//	else if (angle_overshot > 0.01)
//	{
//		new_servo_value = SERVO_LEFT_MAX + servo_offset;
//		__SET_SERVO_TURN(&htim1, new_servo_value);
//		__SET_MOTOR_DIRECTION(DIR_BACKWARD);
//	}
//
//	last_curTask_tick = HAL_GetTick();
//	while(HAL_GetTick() - last_curTask_tick <= 1000){
//		__SET_MOTOR_DUTY(&htim8, 800, 800);
//	}
	osDelay(1);
	if (*targetAngle > 0) {
		if (angleNow > *targetAngle) {
			new_servo_value = SERVO_CENTER - servo_offset;
			__SET_SERVO_TURN(&htim1, new_servo_value);
			__SET_MOTOR_DIRECTION(DIR_BACKWARD);
			last_curTask_tick = HAL_GetTick();
			do {
			  if (HAL_GetTick() - last_curTask_tick >= 30) { // sample gyro every 10ms
				  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
				  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.03;  //
				  __SET_MOTOR_DUTY(&htim8, 1800, 1800);
				  if (angleNow - *targetAngle < -2) break;
				  last_curTask_tick = HAL_GetTick();
			  }
			} while(1);
		} else if (angleNow < *targetAngle) {
			new_servo_value = SERVO_CENTER + servo_offset;
			__SET_SERVO_TURN(&htim1, new_servo_value);
			__SET_MOTOR_DIRECTION(DIR_BACKWARD);
			last_curTask_tick = HAL_GetTick();
			do {
			  if (HAL_GetTick() - last_curTask_tick >= 30) { // sample gyro every 10ms
				  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
				  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.03;  //
				  __SET_MOTOR_DUTY(&htim8, 1800, 1800);
				  if (angleNow - *targetAngle > 2) break;
				  last_curTask_tick = HAL_GetTick();
			  }
			} while(1);
		}
	} else if (*targetAngle < 0) {
		if (angleNow > *targetAngle) {
			new_servo_value = SERVO_CENTER - servo_offset;
			__SET_SERVO_TURN(&htim1, new_servo_value);
			__SET_MOTOR_DIRECTION(DIR_BACKWARD);
			last_curTask_tick = HAL_GetTick();
				do {
				  if (HAL_GetTick() - last_curTask_tick >= 30) { // sample gyro every 10ms
					  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
					  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.03;  //
					  __SET_MOTOR_DUTY(&htim8, 1800, 1800);
					  if (angleNow - *targetAngle < -2) break;
					  last_curTask_tick = HAL_GetTick();
				  }
				} while(1);
		} else if (angleNow < *targetAngle) {
			new_servo_value = SERVO_CENTER + servo_offset;
			__SET_SERVO_TURN(&htim1, new_servo_value);
			__SET_MOTOR_DIRECTION(DIR_BACKWARD);
			last_curTask_tick = HAL_GetTick();
				do {
				  if (HAL_GetTick() - last_curTask_tick >= 30) { // sample gyro every 10ms
					  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
					  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.03;  //
					  __SET_MOTOR_DUTY(&htim8, 1800, 1800);
					  if (angleNow - *targetAngle > 2) break;
					  last_curTask_tick = HAL_GetTick();
				  }
				} while(1);
		}
	}
//	if (angle_overshot > 0.01)
//	{
//
//		new_servo_value = SERVO_LEFT_MAX + servo_offset;
//		__SET_SERVO_TURN(&htim1, new_servo_value);
//		__SET_MOTOR_DIRECTION(DIR_BACKWARD);
//
//	}
//	else if (angle_overshot < -0.01)
//	{
//		new_servo_value = SERVO_RIGHT_MAX - servo_offset;
//		__SET_SERVO_TURN(&htim1, new_servo_value);
//		__SET_MOTOR_DIRECTION(DIR_BACKWARD);
//	}

//	last_curTask_tick = HAL_GetTick();
//	do {
//	  if (HAL_GetTick() - last_curTask_tick >= 20) { // sample gyro every 10ms
//		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ);
//		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;  //
//		  __SET_MOTOR_DUTY(&htim8, 800, 800);
//		  if (angleNow - angle_overshot < 0.01) break;
//		  last_curTask_tick = HAL_GetTick();
//	  }
//	} while(1);



//	if (angle_overshot > 0.01)
//	{
//
//	}
	__SET_MOTOR_DUTY(&htim8, 0, 0);
	__RESET_SERVO_TURN(&htim1);
}

void RobotTurnFastest(float * targetAngle) {
	angleNow = 0; gyroZ = 0;
	last_curTask_tick = HAL_GetTick();
	do {
	  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
		  __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		  angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		  if (abs(angleNow - *targetAngle) < 0.01) break;
		  last_curTask_tick = HAL_GetTick();
	  }
	} while(1);
	__SET_MOTOR_DUTY(&htim8, 0, 0);
	__RESET_SERVO_TURN_FAST(&htim1);
}


void FASTESTPATH_TURN(CmdConfig cfg){
    __SET_CMD_CONFIG(cfg, &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
    
    //    targetDist = 4;
    //    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
}

void FASTESTPATH_TURN_LEFT_90() {
    
    __SET_CMD_CONFIG(cfgs[CONFIG_FP_FL90], &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
//    targetDist = 4;
//    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
    
//	targetAngle = 87.5;
//	__SET_SERVO_TURN(&htim1, 50);
//	__SET_MOTOR_DIRECTION(1);
//	__SET_MOTOR_DUTY(&htim8, 1000, 2000);
//	RobotTurn(&targetAngle);
}

void FASTESTPATH_TURN_RIGHT_90() {

    __SET_CMD_CONFIG(cfgs[CONFIG_FP_FR90], &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
//    targetDist = 4;
//    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
//    osDelay(10);
    
//	targetAngle = -86;
//	__SET_SERVO_TURN(&htim1, 115);
//	__SET_MOTOR_DIRECTION(1);
//	__SET_MOTOR_DUTY(&htim8, 3000, 800);
//
//	RobotTurn(&targetAngle);
}

void FASTESTPATH_TURN_BACK_LEFT_90() {

    __SET_CMD_CONFIG(cfgs[CONFIG_FP_BL90], &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
    targetDist = 4;
    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
    osDelay(10);
    
//    targetAngle = -86;
//    __SET_SERVO_TURN(&htim1, 115);
//    __SET_MOTOR_DIRECTION(1);
//    __SET_MOTOR_DUTY(&htim8, 3000, 800);
//
//    RobotTurn(&targetAngle);
}

void FASTESTPATH_TURN_BACK_RIGHT_90() {

    __SET_CMD_CONFIG(cfgs[CONFIG_FP_BR90], &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
    targetDist = 4;
    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
    osDelay(10);
    
//    targetAngle = -86;
//    __SET_SERVO_TURN(&htim1, 115);
//    __SET_MOTOR_DIRECTION(1);
//    __SET_MOTOR_DUTY(&htim8, 3000, 800);
//
//    RobotTurn(&targetAngle);
}

void FASTESTPATH_TURN_RIGHT_180() {
    
    __SET_CMD_CONFIG(cfgs[CONFIG_FP_FR180], &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
    targetDist = 4;
//    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
//    osDelay(10);
    
//	targetAngle = -176;
//	__SET_SERVO_TURN(&htim1, 115);
//	__SET_MOTOR_DIRECTION(1);
//	__SET_MOTOR_DUTY(&htim8, 3000, 800);
//	RobotTurn(&targetAngle);
}


void FASTESTPATH_TURN_LEFT_180() {
    __SET_CMD_CONFIG(cfgs[CONFIG_FP_FL180], &htim8, &htim1, targetAngle);
    RobotTurnFastest(&targetAngle);
    osDelay(10);
    targetDist = 4;
//    RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
//    osDelay(10);
    
    
//    targetAngle = 177.5;
//    __SET_SERVO_TURN(&htim1, 50);
//    __SET_MOTOR_DIRECTION(1);
//    __SET_MOTOR_DUTY(&htim8, 1000, 2000);
//    RobotTurn(&targetAngle);
}

void RobotMoveUntilIROvershoot() {
	obsDist_IR = 0;
	angleNow = 0; gyroZ = 0;
	  last_curTask_tick = HAL_GetTick();
	  do {
		  __ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		  if (obsDist_IR > 40) break;
		  if (HAL_GetTick() - last_curTask_tick >= 10) {
			  StraightLineMove(SPEED_MODE_2);
			  last_curTask_tick = HAL_GetTick();
		  }
	  } while (1);
	  __SET_MOTOR_DUTY(&htim8, 0, 0);
}

void RobotMoveUntilIRHit() {
	obsDist_IR = 1000;
	angleNow = 0; gyroZ = 0;
	  last_curTask_tick = HAL_GetTick();
	  do {
		  __ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
		  if (obsDist_IR < 40) break;
		  if (HAL_GetTick() - last_curTask_tick >= 10) {
			  StraightLineMove(SPEED_MODE_2);
			  last_curTask_tick = HAL_GetTick();
		  }
	  } while (1);
	  __SET_MOTOR_DUTY(&htim8, 0, 0);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_runOledTask */
/**
  * @brief  Function implementing the oledTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_runOledTask */
void runOledTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//	gyroZPrint=0;previousGyroZPrint=0;angleNowPrint=0;
//		last_curTask_tick = HAL_GetTick();
//		for (;;) {
//			if (HAL_GetTick() - last_curTask_tick >= 20) { // sample gyro every 10ms
//				  __Gyro_Read_Z(&hi2c1, readGyroZDataPrint, gyroZPrint, previousGyroZPrint);
//				  angleNowPrint += gyroZPrint / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;  //
//				  snprintf(ch, sizeof(ch), "angle:%-3.2lf", (float) angleNowPrint);
//				  OLED_ShowString(0, 0, (char *) ch);
//				  snprintf(ch, sizeof(ch), "gyroZ: %" PRIu16, gyroZPrint);
//				  OLED_ShowString(0, 30, (char *) ch);
//				  last_curTask_tick = HAL_GetTick();
//
//			  }
//			snprintf(ch, sizeof(ch), "US_dist: %.2lf" , obsDist_US);
//			OLED_ShowString(0, 50, (char *) ch);
//			OLED_Refresh_Gram();
////			osDelay(250);
//		}
	for(;;){
		snprintf(ch, sizeof(ch), "vert: %d", (int) vertical_distance_to_second_obs);
	    OLED_ShowString(0, 0, (char *) ch);

	    snprintf(ch, sizeof(ch), "horiz: %d", (int) horizontal_dist_bef_turn);
	    OLED_ShowString(0, 30, (char *) ch);



	    OLED_Refresh_Gram();
	    osDelay(250);
	}


//  for(;;)
//  {
////		int16_t gyroZL = 0;
////		int16_t previousGyroZL = 0;
////		float angleNowL = 0;
////		last_curTask_tick = HAL_GetTick();
////		uint8_t readGyroZDataL[2];
////
////		do {
////		  if (HAL_GetTick() - last_curTask_tick >= 10) { // sample gyro every 10ms
////			  __Gyro_Read_Z(&hi2c1, readGyroZDataL, gyroZL, previousGyroZL);
////			  angleNowL += gyroZL / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;  //
////			  tmpAngle = angleNowL;
////			  snprintf(ch, sizeof(ch), "angle:%-3.2lf", (float) angleNowL);
////			  	OLED_ShowString(0, 0, (char *) ch);
////
////			  last_curTask_tick = HAL_GetTick();
////	  		  OLED_Refresh_Gram();
////		  }
////	  	} while(1);
////	sprintf(ch, sizeof(ch), "%-3d%%", (int)batteryVal);
////	int angleTemp = angleNow / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
////	snprintf(ch, sizeof(ch), "angle:%-3.2lf", (float) tmpAngle);
////	OLED_ShowString(0, 0, (char *) ch);
//
//
//
//
////	if (curTask == TASK_FP_XL || curTask == TASK_FP_XR)
////	{
////		snprintf(US_dist, sizeof(US_dist), "dist2: %3.2lf", (float) obsDist_US);
////		OLED_ShowString(0, 20, (char *) US_dist);
////		snprintf(vert_dist, sizeof(vert_dist), "dist2: %3.4lf", (float) vertical_distance_to_second_obs);
////		OLED_ShowString(0, 50, (char *) vert_dist);
////		snprintf(horiz_dist, sizeof(horiz_dist), "dist2: %3.4lf", (float) horizontal_dist_bef_turn);
////		OLED_ShowString(0, 70, (char *) horiz_dist);
////
////	}
////	snprintf(ch, sizeof(ch), "angle:%-3.2lf", (float) angleNowPrint);
////	OLED_ShowString(0, 0, (char *) ch);
////	OLED_Refresh_Gram();
//
////	osDelay(250);
//  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_runCmdTask */
/**
* @brief Function implementing the commandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runCmdTask */
void runCmdTask(void *argument)
{
  /* USER CODE BEGIN runCmdTask */
  /* Infinite loop */
  for(;;)
  {
	  switch(curCmd.index) {
//	  	 case 0: // STOP handled in UART IRQ directly
//	  	  	  break;
	  	 case 1: //FW
	  	 case 2: //BW
	  		curTask = curCmd.index == 1 ? TASK_MOVE : TASK_MOVE_BACKWARD;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	case 3: //FL manual
		case 4: //FR manual
		case 5: //BL manual
		case 6: //BR manual
			__SET_CMD_CONFIG(cfgs[curCmd.index], &htim8, &htim1, targetAngle);
			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			} else __READ_COMMAND(cQueue, curCmd, rxMsg);
			__PEND_CURCMD(curCmd);
			 break;
	  	 case 7: // FL
	  		 curTask = TASK_FL;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 8: // FR
	  		curTask = TASK_FR;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 9: // BL
	  		curTask = TASK_BL;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 10: //BR
	  		curTask = TASK_BR;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 11: // TL
	  	 case 12: // TR
	  		 __SET_SERVO_TURN_MAX(&htim1, curCmd.index - 11 ? 1 : 0);
	  		__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);
	  		 break;
	  	 case 13: // debug IR sensor
	  		 curTask = TASK_ADC;
	  		 break;
	  	 case 14: // DT move until specified distance from obstacle
	  		  curTask = TASK_MOVE_OBS;
	  		  __PEND_CURCMD(curCmd);
	  		 break;
	  	 case 15:
	  		 curTask = TASK_BUZZER;
	  		__PEND_CURCMD(curCmd);
	  		break;
	  	 case 18:
	  		 curTask = TASK_FP_XL;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 19:
	  		 curTask = TASK_FP_XR;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 20:
	  		 curTask = TASK_FP_YL;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 21:
	  		 curTask = TASK_FP_YR;
	  		__PEND_CURCMD(curCmd);
	  		 break;
	  	 case 99:
	  		 break;
	  	 case 100:
	  		 break;
	  	 default:
	  //		 curCmd.index = 99;
	  		 break;
	  	 }

	  osDelay(100);
  }
  /* USER CODE END runCmdTask */
}

/* USER CODE BEGIN Header_runADCTask */
/**
* @brief Function implementing the ADCTask thread.
* @param argument: Not used
* @retval None
* Greedy Task (can only preempted by UART IRQ or EXTI)
* When activate (curTask == TASK_ADC), function executes in 1MHz
*/
/* USER CODE END Header_runADCTask */
void runADCTask(void *argument)
{
  /* USER CODE BEGIN runADCTask */
	uint16_t dataPoint = 0; uint32_t IR_data_raw_acc = 0;
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_ADC) osDelay(1000);
	  else {
//			dataPoint = 0; IR_data_raw_acc = 0; obsDist_IR = 1000;
//			last_curTask_tick = HAL_GetTick();
			__PEND_CURCMD(curCmd);
			targetDist = 40;
			RobotMoveDistObstacle_IR(&targetDist);
//			do {
//				__ADC_Read_Dist(&hadc1, dataPoint, IR_data_raw_acc, obsDist_IR, obsTick_IR);
//			  osDelay(5);
//			} while (1);
//
//		  __ON_TASK_END(&htim8, prevTask, curTask);
//		  HAL_ADC_Stop(&hadc1);
		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);

		} else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }

  }
  /* USER CODE END runADCTask */
}

/* USER CODE BEGIN Header_runMoveDistTask */
/**
* @brief Function implementing the moveDistTask thread.
* @param argument: Not used
* @retval None
* Greedy Task (can only preempted by UART IRQ or EXTI)
*/
/* USER CODE END Header_runMoveDistTask */
void runMoveDistTask(void *argument)
{
  /* USER CODE BEGIN runMoveDistTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_MOVE && curTask != TASK_MOVE_BACKWARD) osDelay(1000);
	  else {
		  if (manualMode) {
			  angleNow = 0; gyroZ = 0; // reset angle for PID
			PIDConfigReset(&pidTSlow);
			PIDConfigReset(&pidSlow);
			PIDConfigReset(&pidFast);

			__SET_MOTOR_DIRECTION(curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD);

			  __ON_TASK_END(&htim8, prevTask, curTask);
			  clickOnce = 0;

			  __CLEAR_CURCMD(curCmd);
			__ACK_TASK_DONE(&huart3, rxMsg);

			last_curTask_tick = HAL_GetTick();
			do {
				if (!manualMode) break;
				if (HAL_GetTick() - last_curTask_tick >= 10) {
					StraightLineMove(SPEED_MODE_T);
					last_curTask_tick = HAL_GetTick();
				}
//				osDelay(5); // for video demo only, give OLED chances to update
			} while (1);

		  } else {
//			  osDelay(5000); // for video demo only
			  targetDist = (float) curCmd.val;
			  // for target distance lesser than 15, move mode must be forced to SLOW
			  if (targetDist <= 15) moveMode = SLOW;

			  if (moveMode == SLOW) {
				  RobotMoveDist(&targetDist, curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_1);
			  } else {
				  RobotMoveDist(&targetDist, curTask == TASK_MOVE ? DIR_FORWARD : DIR_BACKWARD, SPEED_MODE_2);
			  }

			  __ON_TASK_END(&htim8, prevTask, curTask);
				  clickOnce = 0;

				if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
					__CLEAR_CURCMD(curCmd);
					__ACK_TASK_DONE(&huart3, rxMsg);
				} else __READ_COMMAND(cQueue, curCmd, rxMsg);
		  }




	  }
  }
  /* USER CODE END runMoveDistTask */
}

/* USER CODE BEGIN Header_runBuzzerTask */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBuzzerTask */
void runBuzzerTask(void *argument)
{
  /* USER CODE BEGIN runBuzzerTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_BUZZER) osDelay(1000);
	  else {
		  while (curCmd.val > 0) {
			  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
			  osDelay(100);
			  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
			  osDelay(100);
			  curCmd.val--;
		  }
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  clickOnce = 0;


		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			} else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runBuzzerTask */
}

/* USER CODE BEGIN Header_runFLTask */
/**
* @brief Function implementing the FLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFLTask */
void runFLTask(void *argument)
{
  /* USER CODE BEGIN runFLTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_FL) osDelay(1000);
	  else {
//		  osDelay(3000); // video demo only
		  switch(curCmd.val) {
		  case 30: // FL30 (4x2)
			  __SET_CMD_CONFIG(cfgs[CONFIG_FL30], &htim8, &htim1, targetAngle);
//			  RobotTurn(&targetAngle);
			  RobotTurn_V2(&targetAngle);
			  osDelay(10);
			  targetDist = 4;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  break;
		  case 20: // FL20 (outdoor 3x1)
//			  targetDist = 3;
			  targetDist = 1;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_FL20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_FL20]);
			  osDelay(10);
			  targetDist = 8.7;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  default: // FL00 (3x1) CALIBRATED
//			  targetDist = 3;
			  targetDist = 2.5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_FL00], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_FL00]);
			  osDelay(10);
			  targetDist = 10;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;

		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runFLTask */
}

/* USER CODE BEGIN Header_runFRTask */
/**
* @brief Function implementing the FRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFRTask */
void runFRTask(void *argument)
{
  /* USER CODE BEGIN runFRTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_FR) osDelay(1000);
	  else {
//		  osDelay(3000); // video demo only
		  switch(curCmd.val) {
		  case 30: // FR30 (4x2)
			  __SET_CMD_CONFIG(cfgs[CONFIG_FR30], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_FR30]);
			  osDelay(10);
			  targetDist = 4;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  case 20: // FR20 (outdoor 3x1)
			  //targetDist = 3.5 no battery
			  targetDist = 6.9;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_FR20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_FR20]);
			  osDelay(10);
			  targetDist = 13.5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  default: // FR00 (indoor 3x1) CALIBRATED
			  targetDist = 7.5;
			  //targetDist = 3.5 no battery
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_FR00], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_FR00]);
			  osDelay(10);
			  targetDist = 15.5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runFRTask */
}

/* USER CODE BEGIN Header_runBLTask */
/**
* @brief Function implementing the BLTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBLTask */
void runBLTask(void *argument)
{
  /* USER CODE BEGIN runBLTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_BL) osDelay(1000);
	  else {
//		  osDelay(3000); // video demo only
		  switch(curCmd.val) {
		  case 30: // BL30 (4x2)
			  __SET_CMD_CONFIG(cfgs[CONFIG_BL30], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_BL30]);
			  osDelay(10);
			  targetDist = 4.5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  case 20: // BL20 (outdoor 3x1)
			  targetDist = 4.5;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_BL20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_BL20]);
			  osDelay(10);
			  targetDist = 4.5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  default: // BL00 (indoor 3x1)
			  targetDist = 8.5;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_BL00], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_BL00]);
			  osDelay(10);
			  targetDist = 0;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runBLTask */
}

/* USER CODE BEGIN Header_runBRTask */
/**
* @brief Function implementing the BRTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBRTask */
void runBRTask(void *argument)
{
  /* USER CODE BEGIN runBRTask */
  /* Infinite loop */
  for(;;)
  {
	  if (curTask != TASK_BR) osDelay(1000);
	  else {
//		  osDelay(3000); // video demo only
		  switch(curCmd.val) {
		  case 30: // BR30 (4x2)
			  __SET_CMD_CONFIG(cfgs[CONFIG_BR30], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_BR30]);
			  osDelay(10);
			  targetDist = 5;
			  RobotMoveDist(&targetDist, DIR_BACKWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  case 20: // BR20 (outdoor 3x1)
			  //targetDist = 7;
			  targetDist = 13;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_BR20], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_BR20]);
			  osDelay(10);
			  targetDist = 7.3;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  default: // BR00 (indoor 3x1)
			  //targetDist = 7;
			  targetDist = 12.2;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_T);
			  osDelay(10);
			  __SET_CMD_CONFIG(cfgs[CONFIG_BR00], &htim8, &htim1, targetAngle);
			  RobotTurn(&targetAngle, cfgs[CONFIG_BR00]);
			  osDelay(10);
			  targetDist = 6.2;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_T);
			  osDelay(10);
			  break;
		  }


		  clickOnce = 0;
		  prevTask = curTask;
		  curTask = TASK_NONE;
		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runBRTask */
}

/* USER CODE BEGIN Header_runBatteryTask */
/**
* @brief Function implementing the batteryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runBatteryTask */
void runBatteryTask(void *argument)
{
  /* USER CODE BEGIN runBatteryTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,20);
	batteryVal = HAL_ADC_GetValue(&hadc2) / 1421.752066 * 100;
	HAL_ADC_Stop(&hadc2);
    osDelay(30000); // check battery level every 30 seconds
  }
  /* USER CODE END runBatteryTask */
}

/* USER CODE BEGIN Header_runMoveDistObsTask */
/**
* @brief Function implementing the moveDistObsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runMoveDistObsTask */
void runMoveDistObsTask(void *argument)
{
  /* USER CODE BEGIN runMoveDistObsTask */
  /* Infinite loop */
  obsDist_US = 1000;
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);


  for(;;)
  {
	  if (curTask != TASK_MOVE_OBS) osDelay(1000);
	  else {

		  // Step 1. Get US sensor reading
		  last_curTask_tick = HAL_GetTick();
		  while(HAL_GetTick() - last_curTask_tick <= 10){
			  HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
			  __delay_us(&htim4, 10); // wait for 10us
			  HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
			//	  HAL_TIM_Base_Start_IT(&htim4);
			  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
		  }
		  HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);
		  osDelay(10);
		  first_obs_first_stop_dist += obsDist_US;
		  targetDist = (float) curCmd.val;
		  RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);
		  osDelay(300);
		  RPI_ACK_COUNT++;

		  

		  __ON_TASK_END(&htim8, prevTask, curTask);
		  clickOnce = 0;

		if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			__CLEAR_CURCMD(curCmd);
			switch(RPI_ACK_COUNT)
			{
			case 1:
				HAL_UART_Transmit(&huart3, (uint8_t *) "ACK1|\r\n", 7, 0xFFFF); \
				break;
			case 2:
				HAL_UART_Transmit(&huart3, (uint8_t *) "ACK2|\r\n", 7, 0xFFFF); \
				break;
			default:
				break;
			}
//			__ACK_TASK_DONE(&huart3, rxMsg);
		} else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runMoveDistObsTask */
}

/* USER CODE BEGIN Header_runFPFirstObsTurnLeftTask */
/**
* @brief Function implementing the FPFirstObsTurnL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFPFirstObsTurnLeftTask */
void runFPFirstObsTurnLeftTask(void *argument)
{
  /* USER CODE BEGIN runFPFirstObsTurnLeftTask */
  /* Infinite loop */

  // config values
  int left_turn_servo = 30;
  int right_turn_servo = 85;
  float rear_dist_first_obs = 5;

  // gyro measurements
  uint32_t last_tick = 0;
  uint32_t task_start_tick = 0;
  angleNow = 0; gyroZ = 0;



  for(;;)
  {
	  if (curTask != TASK_FP_XL) osDelay(1000);
	  else
	  {
		  /*
		  Steps to execute when robot in front of first obstacle:
		  Step 1: turn left for 300ms
		  Step 2: turn right for 1200ms
		  Step 3: turn left again for 700ms to centralise the car
		  Step 4: Move in front until the car is 50 cm away from the second obstacle
		  */

		    __SET_MOTOR_DIRECTION(DIR_FORWARD);

		    // step 1. turn left for 300ms
		    __SET_MOTOR_DUTY(&htim8, 800, 3000);
		    __SET_SERVO_TURN(&htim1, left_turn_servo);
		    task_start_tick = HAL_GetTick();

		    last_tick = HAL_GetTick();
		    while(HAL_GetTick() - task_start_tick <= 900){
		        if (HAL_GetTick() - last_tick >= 10) { // sample gyro every 10ms
		          __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		          angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		          last_tick = HAL_GetTick();
		        }
		    }

		    //step 2. turn right for 1200ms
		    __SET_MOTOR_DUTY(&htim8, 3000, 800);
			__SET_SERVO_TURN(&htim1, right_turn_servo);
		    task_start_tick = HAL_GetTick();

		    last_tick = HAL_GetTick();
		    while(HAL_GetTick() - task_start_tick <= 1500){
		        if (HAL_GetTick() - last_tick >= 10) { // sample gyro every 10ms
		          __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		          angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		          last_tick = HAL_GetTick();
		        }
		    }

		    //step 3. turn left again for 700ms to centralise the car
		    __SET_MOTOR_DUTY(&htim8, 800, 3000);
		    __SET_SERVO_TURN(&htim1, left_turn_servo);
		    task_start_tick = HAL_GetTick();

		    last_tick = HAL_GetTick();
		    while(HAL_GetTick() - task_start_tick <= 1600){
		        if (HAL_GetTick() - last_tick >= 10) { // sample gyro every 10ms
		          __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		          angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		          last_tick = HAL_GetTick();
		        }
		        if (angleNow < 20){
		        	__SET_MOTOR_DUTY(&htim8, (int) (0.6*800), (int) (0.6*3000));
		        }
		    }

		    __SET_MOTOR_DUTY(&htim8, 0, 0);
		    __RESET_SERVO_TURN(&htim1);


		    /*
		     step 4. If obs is within 20cm from car, move back 40cm.
		     Else, move to 40 cm before obs.
		     Also note down the distance from car to second obs after turn.
		     */

		    vertical_distance_to_second_obs += first_obs_first_stop_dist + rear_dist_first_obs;

		    // shoot US sensor to determine second obstacle distance
		    obsDist_US = 0;
		    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
		    last_curTask_tick = HAL_GetTick();
		    while(HAL_GetTick() - last_curTask_tick <= 10){
		    	HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		    	__delay_us(&htim4, 10); // wait for 10us
		    	HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
		    	//	  HAL_TIM_Base_Start_IT(&htim4);
		    	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
		    }
		    HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);
		    osDelay(10);
		    second_obs_dist = obsDist_US;
		    vertical_distance_to_second_obs += obsDist_US;

		    // Move to within 50cm from second obstacle
		    targetDist = 40;
		    if(obsDist_US < 20){
		    	RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);
		    	vertical_distance_to_second_obs -= 40 - obsDist_US;
		    }else{
		    	targetDist = 50;
		    	RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);
		    }


		  __ON_TASK_END(&htim8, prevTask, curTask);

		  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
			  __CLEAR_CURCMD(curCmd);
			  __ACK_TASK_DONE(&huart3, rxMsg);
		  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
	  }
  }
  /* USER CODE END runFPFirstObsTurnLeftTask */
}

/* USER CODE BEGIN Header_runFPFirstObsTurnRightTask */
/**
* @brief Function implementing the FPFirstObsTurnR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFPFirstObsTurnRightTask */
void runFPFirstObsTurnRightTask(void *argument)
{
  /* USER CODE BEGIN runFPFirstObsTurnRightTask */
	/* Infinite loop */

    // config values
    int left_turn_servo = 30;
    int right_turn_servo = 75;
    float rear_dist_first_obs = 5;

    // gyro measurements
    uint32_t last_tick = 0;
    uint32_t task_start_tick = 0;
    angleNow = 0; gyroZ = 0;

	for(;;)
	{
		if (curTask != TASK_FP_XR) osDelay(1000);
		else
		{
			  /*
			Steps to execute when robot in front of first obstacle:
			Step 1: turn right for 300ms
			Step 2: turn left for 1200ms
			Step 3: turn right again for 700ms to centralise the car
			Step 4: Move in front until the car is 50 cm away from the second obstacle
			*/

		    __SET_MOTOR_DIRECTION(DIR_FORWARD);

		    // step 1. turn right for 300ms
		    __SET_MOTOR_DUTY(&htim8, 3000, 800);
		    __SET_SERVO_TURN(&htim1, right_turn_servo);
		    task_start_tick = HAL_GetTick();

		    last_tick = HAL_GetTick();
		    while(HAL_GetTick() - task_start_tick <= 900){
		        if (HAL_GetTick() - last_tick >= 10) { // sample gyro every 10ms
		          __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		          angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		          last_tick = HAL_GetTick();
		        }
		    }

		    //step 2. turn left for 1200ms
		    __SET_MOTOR_DUTY(&htim8, 800, 3000);
			__SET_SERVO_TURN(&htim1, left_turn_servo);
		    task_start_tick = HAL_GetTick();

		    last_tick = HAL_GetTick();
		    while(HAL_GetTick() - task_start_tick <= 1500){
		        if (HAL_GetTick() - last_tick >= 10) { // sample gyro every 10ms
		          __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		          angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		          last_tick = HAL_GetTick();
		        }
		    }

		    //step 3. turn right again for 700ms to centralise the car
		    __SET_MOTOR_DUTY(&htim8, 3000, 800);
		    __SET_SERVO_TURN(&htim1, 85);
		    task_start_tick = HAL_GetTick();

		    last_tick = HAL_GetTick();
		    while(HAL_GetTick() - task_start_tick <= 1600){
		        if (HAL_GetTick() - last_tick >= 10) { // sample gyro every 10ms
		          __Gyro_Read_Z(&hi2c1, readGyroZData, gyroZ, previousGyroZ);
		          angleNow += gyroZ / GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS * 0.01;
		          last_tick = HAL_GetTick();
		        }
		        if (angleNow < 20){
		        	__SET_MOTOR_DUTY(&htim8, (int) (0.6*3000), (int) (0.6*800));
		        }
		    }

		    __RESET_SERVO_TURN(&htim1);
		    __SET_MOTOR_DUTY(&htim8, 0, 0);

		    /*
		     step 4. If obs is within 20cm from car, move back 40cm.
		     Else, move to 40 cm before obs.
		     Also note down the distance from car to second obs after turn.
		     */

		    vertical_distance_to_second_obs += first_obs_first_stop_dist + rear_dist_first_obs;

		    // shoot US sensor to determine second obstacle distance
		    obsDist_US = 0;
		    HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
		    last_curTask_tick = HAL_GetTick();
		    while(HAL_GetTick() - last_curTask_tick <= 10){
		    	HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		    	__delay_us(&htim4, 10); // wait for 10us
		    	HAL_GPIO_WritePin(TRI_GPIO_Port, TRI_Pin, GPIO_PIN_RESET);  // pull the TRIG pin low
		    	//	  HAL_TIM_Base_Start_IT(&htim4);
		    	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC2);
		    }
		    HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_2);
		    osDelay(10);
		    second_obs_dist = obsDist_US;
		    vertical_distance_to_second_obs += obsDist_US;

		    // Move to within 50cm from second obstacle
		    targetDist = 40;
		    if(obsDist_US < 20){
		    	RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);
		    	vertical_distance_to_second_obs -= 40 - obsDist_US;
		    }else{
		    	targetDist = 50;
		    	RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);
		    }

			 __ON_TASK_END(&htim8, prevTask, curTask);

			if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				__CLEAR_CURCMD(curCmd);
				__ACK_TASK_DONE(&huart3, rxMsg);
			} else __READ_COMMAND(cQueue, curCmd, rxMsg);
		}
	}
  /* USER CODE END runFPFirstObsTurnRightTask */
}

/* USER CODE BEGIN Header_runFPSecondObsTurnLeftTask */
/**
* @brief Function implementing the FPScndObsTurnL thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFPSecondObsTurnLeftTask */
void runFPSecondObsTurnLeftTask(void *argument)
{
  /* USER CODE BEGIN runFPSecondObsTurnLeftTask */
  /* Infinite loop */
	// CREATE THESE GLOBAL VARIABLES
	  // const float HORIZONTAL_TURN_RADIUS
	  // const float VERTICAL_TURN_RADIUS
	  // float horizontal_dist_bef_turn = 0;
	  // float vertical_distance_to_second_obs = 0; //supposed to use as global variable

	  // THINGS TO CHECK
	  // CHECK IF NEED TO USE BR30 OR OTHER BL FORMS for the Backward right movement


	  osDelay(500); //give a short while for US to update value
//	  float dist_second_obs = obsDist_US; // record the dist of second obs from the car before executing turn

	  for(;;)
	  {
		  if (curTask != TASK_FP_YL) osDelay(1000);
		  else
		  {
			  /*
			  Steps to execute when robot in front of SECOND obstacle
			  Step 1: turn left 90 degree
			  Step 2: Move straight for 30 cm
			  Step 3: Turn right 90 Degree. Check using Ultrasonic sensor
			  if the obstacle is still in front
			  Step 4: If obstacle is still in front, come BACK RIGHT +++++ MOVE STRAIGHT FOR EXTRA 20CM
			  and repeat Step 3.
			  Step 5: If obstacle is not in front, Turn right 90 Degree.
			  Step 6: Move Straight across obstacle distance to cross over
			  Step 7: Turn Right 90 Degrees
			  Step 8: Move Straight until in line with first obstacle
			  Step 9: Turn Right 90 Degrees
			  Step 10: Move forward until before turn radius to center
			  Step 11: Turn Left 90 Degrees to face the parking spot
			  Step 12: Move until 5 cm before parking spot using UltraSonic sensor
			  */

			  //  Step 1: turn left 90 degree
			  FASTESTPATH_TURN_LEFT_90();
			  horizontal_dist_bef_turn += HORIZONTAL_TURN_RADIUS;
//			  vertical_distance_to_second_obs += VERTICAL_TURN_RADIUS;
			  osDelay(10);

			  // Step 2: Move straight for 30 cm
			  targetDist = 50;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  horizontal_dist_bef_turn += 50;
			  osDelay(10);

			  // Step 3: Turn right 180 Degree.
			  FASTESTPATH_TURN_RIGHT_180();
			  horizontal_dist_bef_turn -= 3;
			  vertical_distance_to_second_obs += 15;
			  osDelay(300); //give a short while for US to update value


//			  // Step 4: If obstacle is still in front, come BACK RIGHT +++++ MOVE STRAIGHT FOR EXTRA 20CM
//			  // Repeat this until robot not in front of a obstacle
//			  while(obsDist_US <= dist_second_obs)
//			  {
//				// 4.1 Move back right
//                FASTESTPATH_TURN_BACK_RIGHT_90();
//				osDelay(10);
//
//				// determine what's the horizontal dist subtracted when moving BACK RIGHT
//				// will set it to 10 for now
//				vertical_distance_to_second_obs -= HORIZONTAL_TURN_RADIUS;
//				horizontal_dist_bef_turn -= VERTICAL_TURN_RADIUS;
//
//				// 4.2 Go in front 20 cm
//				targetDist = 20;
//                RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
//				horizontal_dist_bef_turn += 20;
//                osDelay(10);
//
//				// 4.3 Turn right
//				FASTESTPATH_TURN_RIGHT_90();
//				horizontal_dist_bef_turn += VERTICAL_TURN_RADIUS;
//				vertical_distance_to_second_obs += HORIZONTAL_TURN_RADIUS;
//				osDelay(300);
//			  }


			  // use below if need to move in front of obstacle
			  // targetDist = 20;
					// RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
					// osDelay(10);

//
//			  // Step 5: If obstacle is not in front, Turn right 90 Degree.
//			  FASTESTPATH_TURN_RIGHT_90();
//			  horizontal_dist_bef_turn -= HORIZONTAL_TURN_RADIUS;
//			  vertical_distance_to_second_obs += VERTICAL_TURN_RADIUS;
//			  osDelay(10);

			  // Step 6: Move Straight across obstacle distance to cross over
			  targetDist = horizontal_dist_bef_turn * 2;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  osDelay(10);
			  //// HORIZONTAL DISPLACEMENT NOT UPDATED AS WE'RE ASSUMING WE ARE TRAVELLING ACCURATELY AND THAT
		          //// THE DISPLACEMENT WILL BE THE SAME

			  // Step 7: Turn Right 90 Degrees
			  FASTESTPATH_TURN_RIGHT_90();
			  horizontal_dist_bef_turn += VERTICAL_TURN_RADIUS;
			  vertical_distance_to_second_obs -= HORIZONTAL_TURN_RADIUS;
			  osDelay(10);

			  // Step 8: Move Straight until 60cm before the parking spot
			  targetDist = vertical_distance_to_second_obs - 60;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  osDelay(10);
			  // vertical_distance_to_second_obs -= 50 -- need to care about vert dist?


			  // Step 9: Turn Right 90 Degrees
			  FASTESTPATH_TURN_RIGHT_90();
			  horizontal_dist_bef_turn -= HORIZONTAL_TURN_RADIUS;
              vertical_distance_to_second_obs -= VERTICAL_TURN_RADIUS;
			  osDelay(10);
			  // vertical_distance_to_second_obs -= VERTICAL_TURN_RADIUS -- need to care about vert dist?

			  // Step 10: Move forward until before turn radius to center
			  targetDist = horizontal_dist_bef_turn - 15;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  osDelay(10);
			  // vertical_distance_to_second_obs -= 50 -- need to care about vert dist?

			  // Step 11: Turn Left 90 Degrees to face the parking spot
			  FASTESTPATH_TURN_LEFT_90();
              horizontal_dist_bef_turn -= VERTICAL_TURN_RADIUS;
              vertical_distance_to_second_obs -= HORIZONTAL_TURN_RADIUS;
			  osDelay(10);
			  // horizontal_dist_bef_turn -= HORIZONTAL_TURN_RADIUS; -- need to care about horizontal dist?
			  // vertical_distance_to_second_obs -= VERTICAL_TURN_RADIUS -- need to care about vert dist?

			  // Step 12: Move until 5 cm before parking spot using UltraSonic sensor
			  targetDist = 5;
			  RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);


			  __ON_TASK_END(&htim8, prevTask, curTask);

			  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
				  __CLEAR_CURCMD(curCmd);
				  __ACK_TASK_DONE(&huart3, rxMsg);
			  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
		  }
	  }
  /* USER CODE END runFPSecondObsTurnLeftTask */
}

/* USER CODE BEGIN Header_runFPSecondObsTurnRightTask */
/**
* @brief Function implementing the FPScndObsTurnR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_runFPSecondObsTurnRightTask */
void runFPSecondObsTurnRightTask(void *argument)
{
  /* USER CODE BEGIN runFPSecondObsTurnRightTask */
  /* Infinite loop */

	// CREATE THESE GLOBAL VARIABLES
	// const float HORIZONTAL_TURN_RADIUS
	// const float VERTICAL_TURN_RADIUS
	// float horizontal_dist_bef_turn = 0;
	// float vertical_distance_to_second_obs = 0; //supposed to use as global variable

	// THINGS TO CHECK
	// CHECK IF NEED TO USE BL30 OR OTHER BL FORMS

	  osDelay(300); //give a short while for US to update value
//	  float dist_second_obs = obsDist_US; // record the dist of second obs from the car before executing turn

	  for(;;)
	  {
		  if (curTask != TASK_FP_YR) osDelay(1000);
		  else
		  {
			  /*
			  Steps to execute when robot in front of SECOND obstacle
			  Step 1: turn right 90 degree
			  Step 2: Move straight for 30 cm
			  Step 3: Turn left 90 Degree. Check using Ultrasonic sensor
			  if the obstacle is still in front
			  Step 4: If obstacle is still in front, come BACK LEFT +++++ MOVE STRAIGHT FOR EXTRA 20CM
			  and repeat Step 3.
			  Step 5: If obstacle is not in front, Turn left 90 Degree.
			  Step 6: Move Straight across obstacle distance to cross over
			  Step 7: Turn Left 90 Degrees
			  Step 8: Move Straight until in line with first obstacle
			  Step 9: Turn Left 90 Degrees
			  Step 10: Move forward until before turn radius to center
			  Step 11: Turn Right 90 Degrees to face the parking spot
			  Step 12: Move until 5 cm before parking spot using UltraSonic sensor
			  */

			  //  Step 1: turn right 90 degree
			  FASTESTPATH_TURN_RIGHT_90();
              horizontal_dist_bef_turn += HORIZONTAL_TURN_RADIUS;
//              vertical_distance_to_second_obs += VERTICAL_TURN_RADIUS;
			  osDelay(10);

			  // Step 2: Move straight for 50 cm
			  targetDist = 50;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  horizontal_dist_bef_turn += 50;
			  osDelay(10);

			  // Step 3: Turn right 180 Degree.
			  FASTESTPATH_TURN_LEFT_180();
			  horizontal_dist_bef_turn -= 3;
			  vertical_distance_to_second_obs += 15;
			  osDelay(300); //give a short while for US to update value


			  // Step 4: If obstacle is still in front, come BACK LEFT +++++ MOVE STRAIGHT FOR EXTRA 20CM
			  // and repeat Step 3.
//			  while(obsDist_US <= dist_second_obs)
//			  {
//				// 4.1 Move back left
//                FASTESTPATH_TURN_BACK_LEFT_90();
//                  vertical_distance_to_second_obs -= HORIZONTAL_TURN_RADIUS;
//                  horizontal_dist_bef_turn -= VERTICAL_TURN_RADIUS;
//                  osDelay(10);
//
//				// 4.2 Go in front 20 cm
//				targetDist = 20;
//					  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
//				 horizontal_dist_bef_turn += 20;
//					  osDelay(10);
//
//				// 4.3 Turn left
//				FASTESTPATH_TURN_LEFT_90();
//                horizontal_dist_bef_turn += VERTICAL_TURN_RADIUS;
//                vertical_distance_to_second_obs += HORIZONTAL_TURN_RADIUS;
//				osDelay(300);
//			  }


			  // use below if need to move in front of obstacle
			  // targetDist = 20;
					// RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
					// osDelay(10);


			  // Step 5: If obstacle is not in front, Turn left 90 Degree.
//			  FASTESTPATH_TURN_LEFT_90();
//			  horizontal_dist_bef_turn -= HORIZONTAL_TURN_RADIUS;
//			  vertical_distance_to_second_obs += VERTICAL_TURN_RADIUS;
//			  osDelay(10);

			  // Step 6: Move Straight across obstacle distance to cross over
			  targetDist = horizontal_dist_bef_turn * 2;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  osDelay(10);

			  // Step 7: Turn Left 90 Degrees
			  FASTESTPATH_TURN_LEFT_90();
              horizontal_dist_bef_turn += VERTICAL_TURN_RADIUS;
              vertical_distance_to_second_obs -= HORIZONTAL_TURN_RADIUS;
			  osDelay(10);

              // Step 8: Move Straight until 60cm before the parking spot
              targetDist = vertical_distance_to_second_obs - 60;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  osDelay(10);

			  // Step 9: Turn Left 90 Degrees
			  FASTESTPATH_TURN_LEFT_90();
              horizontal_dist_bef_turn -= HORIZONTAL_TURN_RADIUS;
              vertical_distance_to_second_obs -= VERTICAL_TURN_RADIUS;
			  osDelay(10);
			  // vertical_distance_to_second_obs -= VERTICAL_TURN_RADIUS -- need to care about vert dist?

			  // Step 10: Move forward until before turn radius to center
              targetDist = horizontal_dist_bef_turn - 15;
			  RobotMoveDist(&targetDist, DIR_FORWARD, SPEED_MODE_2);
			  osDelay(10);
			  // vertical_distance_to_second_obs -= 50 -- need to care about vert dist?

			  // Step 11: Turn Right 90 Degrees to face the parking spot
			  FASTESTPATH_TURN_RIGHT_90();
              horizontal_dist_bef_turn -= VERTICAL_TURN_RADIUS;
              vertical_distance_to_second_obs -= HORIZONTAL_TURN_RADIUS;
			  osDelay(10);
			  // horizontal_dist_bef_turn -= HORIZONTAL_TURN_RADIUS; -- need to care about horizontal dist?
			  // vertical_distance_to_second_obs -= VERTICAL_TURN_RADIUS -- need to care about vert dist?

			  // Step 12: Move until 5 cm before parking spot using UltraSonic sensor
			  targetDist = 5;
			  RobotMoveDistObstacle(&targetDist, SPEED_MODE_2);


			  __ON_TASK_END(&htim8, prevTask, curTask);

			  if (__COMMAND_QUEUE_IS_EMPTY(cQueue)) {
					__CLEAR_CURCMD(curCmd);
					__ACK_TASK_DONE(&huart3, rxMsg);
			  } else __READ_COMMAND(cQueue, curCmd, rxMsg);
		  }
	  }

  /* USER CODE END runFPSecondObsTurnRightTask */
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
