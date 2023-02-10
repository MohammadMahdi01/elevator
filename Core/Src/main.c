/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {down=-1, fixed,up} direction;
typedef enum {false, true} bool;
typedef enum {off, on} key;
typedef struct
{
    uint16_t frequency;
    uint16_t duration;
} Tone;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MINWAIT 500
#define MAXWAIT 5000
#define QMINLENGTH 1
#define QMAXLENGTH 9
#define MINMELODYNUM 1
#define MAXMELODYNUM MINMELODYNUM + melodyCount -1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
key alarmed=off;
key alarmedLED=on;
key alarmedLEDEntry=on;
key alarmedLEDTurn=on;
key music=on;
key musicEntry=on;
key adminPanel=false;
direction dir=fixed;
int i=0;
int last=0;
int curLevel=0;
int minLevel=0;
int maxLevel=9;
int pass=123;
int passEntry;
int maxLevelEntry=9;
int curLevelEntry=9;
int waitEntry=900;
int extraCommonLevels=0;
int melodyCount=2;
int melodyNum=1;
int melodyNumEntry=1;
int curDest=0;
int newDest=0;
int qLength=0;
int digitIndex=0;
int waitDuration=2000;
TIM_HandleTypeDef *wait_timer = &htim3;	// Point to PWM Timer configured in CubeMX
int levelsQueue[QMAXLENGTH];
int levelsQueueEntry[QMAXLENGTH];//NULL
char input[20];//"20c"
char sample[QMAXLENGTH];
//int sample;
char OTest[5];
char output[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
int round100(int);
char* keyName(int keyValue);
void selectionSort(int[], int);
char* substring(char*, int, int);
void setMelody(int i);
extern volatile uint16_t volume;
extern Tone super_mario_bros[];
extern Tone harrypotter[];
extern int super_mario_bros_size;
extern int harrypotter_size;
extern void PWM_Start();
extern void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume);
extern void Change_Melody(const Tone *melody, uint16_t tone_count);
extern void Update_Melody();
extern void numberToBCD(int i);
extern void showSS(int i);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void testNum(char text[50],int num){
	char OPTest[50];
	sprintf(OPTest, "%s:%d\n", text,num);
	HAL_UART_Transmit(&huart1, OPTest, strlen(OPTest),100);
}
void deQueue(){
	curDest=levelsQueue[0];
	qLength--;
	for(int i=0;i<qLength;i++)
		levelsQueue[i]=levelsQueue[i+1];
	levelsQueue[qLength]=NULL;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8<<i%8, 1);
	i++;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
    	//Test
    	//sprintf(output, "%d-", curLevel);
	    //HAL_UART_Transmit(&huart1, output, strlen(output),100);

    	//Seven Segment
    	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, 1);
    	  if(!(alarmed && !alarmedLEDTurn)){
    		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_0<<(digitIndex+1),0);
    	  }

    	  showSS(digitIndex+1);
    	  digitIndex++;
    	  digitIndex%=4;

    	//LED
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,alarmed && alarmedLED && alarmedLEDTurn);

		//Direction
			dir=(curDest<=curLevel)?((curDest<curLevel)?-1:0):1;

    	//Buzzer
		if(alarmed){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8<<0,2);
			const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
			const uint32_t period_cycles = (waitDuration<2000)?10000:20000;
			const uint32_t prescaler =  internal_clock_freq/1000*waitDuration / period_cycles ;
			wait_timer->Instance->PSC = prescaler -1 ;
			wait_timer->Instance->ARR = period_cycles;
			}
		else if(music){
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8<<0,1);
			  Update_Melody();
		}
		}

    if (htim->Instance == TIM3)
    {

    	if(curLevel==curDest){
    		if(qLength!=0){
    			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8<<i%8, 1);
    			i++;
    			deQueue();
    		}
    	}
    	else{
    		(curLevel<curDest)?curLevel++:curLevel--;
    	}

    }
    if (htim->Instance == TIM4)
    {

    	alarmedLEDTurn=!alarmedLEDTurn;
    	volume=volume?0:3;
    	if(alarmed)
    	PWM_Change_Tone(300,volume);
    }

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GetTick()-last<100){
		  return;
	  }

	else if (GPIO_Pin == GPIO_PIN_0){
    	}
	else if (GPIO_Pin == GPIO_PIN_1 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)){
    	alarmed=!alarmed;
		sprintf(output, "Alarm button pressed!-->Alarm state is[%s]!\n", keyName(alarmed));
    	PWM_Change_Tone(300,0);
    	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);
    }
	else if (GPIO_Pin == GPIO_PIN_2 && HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)){
		if(qLength<5){
			int qIndex=0;
    		for(qIndex=0;qIndex<QMAXLENGTH;qIndex++)
    			if(newDest==levelsQueue[qIndex-1]){
    				sprintf(output, "Assign button pressed!-->new destination[%d] exists in level queue!\n", newDest);
    				return;
    			}
    		if(qIndex==QMAXLENGTH){
    			sprintf(output, "Assign button pressed!-->new destination[%d] assigned to level queue successfully!\n", newDest);
    			levelsQueue[qLength++]=newDest;
    			selectionSort(levelsQueue,qLength);
    			for(qIndex=0;qIndex<qLength/2;qIndex++){
    				if(levelsQueue[qIndex]<curDest){
    					int temp=levelsQueue[qIndex];
    					levelsQueue[qIndex]=levelsQueue[qLength-1];
    					levelsQueue[qLength-1]=temp;
    				}

    			}
    		}
		}
    }
	else if (GPIO_Pin == GPIO_PIN_3 ){
    	if(0<newDest){
    		newDest--;
    		sprintf(output, "decrease level button pressed!-->new potential destination is[%d]!\n", newDest);
    	}
    	else{
    		sprintf(output, "decrease level button pressed!-->potential destination had already reached to its minimum level[%d]!\n", minLevel);
    	}

	}
	else if (GPIO_Pin == GPIO_PIN_4){
		if(newDest<maxLevel){
			newDest++;
		    sprintf(output, "Increase level button pressed!-->new potential destination is[%d]!\n", newDest);
		}
		else{
		    sprintf(output, "Increase level button pressed!-->potential destination had already reached to its maximum level[%d]!\n", maxLevel);
		 }

	}
	HAL_UART_Transmit(&huart1, output, strlen(output),100);
    last=HAL_GetTick();
}
/*ADMIN#Pass065 068 077 073 078 035 049 050 051 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 *SET MAX LEVEL [99]083 069 084 032 077 065 088 032 076 069 086 069 076 032 091 057 057 093 000 000 000 000 000 000 000 000 000 000
 *SET LEVEL [9]083 069 084 032 076 069 086 069 076 032 091 057 093 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 *SET WAIT [900]083 069 084 032 087 065 073 084 032 091 057 048 048 093 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 *SET LED OFF083 069 084 032 076 069 068 032 079 070 070 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 *SET MUSIC OFF083 069 084 032 076 069 068 032 079 070 070 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 *SET MELODY [2]083 069 084 032 077 069 076 079 068 089 032 091 050 093 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 *TEST#sample084 069 083 084 035 050 048 051 049 057 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000 000
 * */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	if(qLength==0){
    		if(!strcmp(substring(input, 1, 6),"ADMIN#")){
    		    //strncpy(sample, substring(input, 7, 14), 14);
    		    passEntry = atoi(substring(input, 7, 14));
    		    if(adminPanel){
    				sprintf(output, "You have already entered the admin panel successfully!\n");
    			}
    		    else{
    		    	if(passEntry==pass){
    		    		adminPanel=true;
    		    	    sprintf(output, "You entered the admin panel successfully!\n");
    		    	}else{
    		    		sprintf(output, "Wrong password!\n");
    		    	}
    		    }
    		}
    		else{
    			sprintf(output, "Please Enter the admin panel first!\n");
    		}
    		if(adminPanel){
				if(!strcmp(substring(input, 1, 4),"SET ")){
					if(!strcmp(substring(input, 5, 10),"MAX LEVEL ")){
					  //not string

						maxLevelEntry = atoi(substring(input, 16, 5));
						//sscanf(substring(input, 16, 1), "%d", &maxLevelEntry);
						if(maxLevelEntry<0){
							sprintf(output, "New max level[%d] is Less Than 0!\n", maxLevelEntry);
						}
						else if(9<maxLevelEntry){
							sprintf(output, "New max level[%d] is more Than 9!\n", maxLevelEntry);
						}
						else{
							sprintf(output, "New max level[%d] assigned successfully!\n", maxLevelEntry);
							maxLevel=maxLevelEntry;
						}

						//sprintf(OTest, "%d\n", maxLevel);
						//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8<<maxLevel-5, 1);
						//HAL_UART_Transmit(&huart1, OTest, sizeof(OTest),100);
					}
					else if(!strcmp(substring(input, 5, 6),"LEVEL ")){
						curLevelEntry = atoi(substring(input, 12, 9));
						if(curLevelEntry<0){
							sprintf(output, "New level[%d] is Less Than max level[%d]!\n", curLevelEntry, maxLevel);
						}
						else if(maxLevel<curLevelEntry){
							sprintf(output, "New level[%d] is more Than max level[%d]!\n", curLevelEntry, maxLevel);
						}
						else{
							sprintf(output, "New level[%d] assigned successfully!\n", curLevelEntry);
							curLevel=curLevelEntry;
							curDest=curLevel;
						}
					}
					else if(!strcmp(substring(input, 5, 5),"WAIT ")){
						waitEntry = atoi(substring(input, 11, 10));
						if(waitEntry<MINWAIT){
							sprintf(output, "New wait[%d] is Less Than min wait[%d]!\n", waitEntry, MINWAIT);
						}
						else if(MAXWAIT<waitEntry){
							sprintf(output, "New wait[%d] is more Than max wait[%d]!\n", waitEntry, MAXWAIT);
						}
						else{
							if(waitEntry%100==0)
								sprintf(output, "New wait[%d] assigned successfully!\n", waitEntry);
							else
								sprintf(output, "Approximated value[%d] from new wait[%d] assigned successfully!\n",round100(waitEntry), waitEntry);
							waitDuration=waitEntry;
						}
					}
					else if(!strcmp(substring(input, 5, 4),"LED ")){
						alarmedLEDEntry=strcmp(substring(input, 9, 2),"OF")?1:0;
						sprintf(output, "New alarmed LED state[%s] assigned to old alarmed LED state[%s] successfully!\n", keyName(alarmedLEDEntry), keyName(alarmedLED));
						//testNum("alarmedLEDEntry",alarmedLEDEntry);
						//testNum("alarmedLED",alarmedLED);
						alarmedLED=alarmedLEDEntry;

						}
					else if(!strcmp(substring(input, 5, 6),"MUSIC ")){
						musicEntry=strcmp(substring(input, 11, 2),"OF")?1:0;
						sprintf(output, "New MUSIC state[%s] assigned to old MUSIC state[%s] successfully!\n", keyName(musicEntry), keyName(music));
						//testNum("musicEntry",musicEntry);
						//testNum("music",music);
						music=musicEntry;
						PWM_Change_Tone(0,0);
						}
					else if(!strcmp(substring(input, 5, 7),"MELODY ")){
						melodyNumEntry = atoi(substring(input, 13, 8));
						if(melodyNumEntry<MINMELODYNUM){
							sprintf(output, "New melody number[%d] is Less Than min melody number[%d]!\n", melodyNumEntry, MINMELODYNUM);
						}
						else if(MAXMELODYNUM<melodyNumEntry){
							sprintf(output, "New melody number[%d] is more Than max melody number[%d]!\n", melodyNumEntry, MAXMELODYNUM);
							}
						else if(melodyNumEntry==melodyNum){
							sprintf(output, "New melody number[%d] is same as old melody number[%d]!\n", melodyNumEntry, melodyNum);
						}
						else{
							sprintf(output, "New melody number[%d] assigned to old melody number[%d] successfully!!\n", melodyNumEntry, melodyNum);
							melodyNum=melodyNumEntry;
							setMelody(melodyNum);
						}
						//testNum("musicEntry",musicEntry);
						//testNum("music",music);
						music=musicEntry;
						PWM_Change_Tone(0,0);
						}
				}
				else if(!strcmp(substring(input, 1, 5),"TEST#")){
					strncpy(sample, substring(input, 6, 15), 15);
					if(strlen(sample)<QMINLENGTH){
						sprintf(output, "New sample queue[%s] size[%d] is Less Than queue min length[%d]!\n", sample,strlen(sample), QMINLENGTH);
					}
					else if(QMAXLENGTH<strlen(sample)){
						sprintf(output, "New sample queue[%s] size[%d] is more Than queue max length[%d]!\n", sample,strlen(sample), QMAXLENGTH);
					}
					else{
						for(int i=0;i<strlen(sample);i++){
							sscanf(substring(sample, i+1, 1), "%d", &levelsQueueEntry[i]);
						}
						selectionSort(levelsQueueEntry, strlen(sample));
						extraCommonLevels=0;
						for(int i=1;i<strlen(sample);i++){
							if(levelsQueueEntry[i-1]==levelsQueueEntry[i]){
								extraCommonLevels++;
							}
						}
						if(extraCommonLevels){
							sprintf(output, "New sample queue[%s] has [%d] extra common levels!\n", sample,extraCommonLevels);
						}
						else{
							sprintf(output, "New sample queue[%s] assigned successfully!\n", sample,extraCommonLevels);
							qLength=strlen(sample);
							for(int i=0;i<qLength;i++){
								levelsQueue[i]=levelsQueueEntry[i];
								//testNum("levelsQueue[i]",levelsQueue[i]);
							}
							//testNum("levelsQueue[0]",qLength);
							selectionSort(levelsQueue,qLength);

						}
					}
					//waitEntry = atoi(substring(input, 6, 5));

				}
			}
    	}
    	else{
    		sprintf(output, "Please wait until the queue gets empty!\n", maxLevelEntry);
    	}
    	HAL_UART_Transmit(&huart1, output, strlen(output),100);
        //HAL_UART_Receive_IT(&huart1, input, 20);

    }
    //HAL_UART_Receive_IT(&huart1, input, 20);
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);
	PWM_Start();
	setMelody(1);
	HAL_UART_Receive_IT(&huart1, input, sizeof(input));
	//testNum("volume",volume);
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 143;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT4_Pin */
  GPIO_InitStruct.Pin = MEMS_INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD1 PD2 PD3 PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
char *substring(char *string, int position, int length)
{
   char *p;
   int c;

   p = malloc(length+1);

   if (p == NULL)
   {
      printf("Unable to allocate memory.\n");
      exit(1);
   }

   for (c = 0; c < length; c++)
   {
      *(p+c) = *(string+position-1);
      string++;
   }

   *(p+c) = '\0';

   return p;
}
int round100(int n){
	return 100*(n/100+((n%100)/50));
}
char* keyName(int keyValue){
	return keyValue?"ON":"OFF";
}
void swap(int* xp, int* yp)
{
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Function to perform Selection Sort
void selectionSort(int arr[], int n)
{
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {

        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx])
                min_idx = j;

        // Swap the found minimum element
        // with the first element
        swap(&arr[min_idx], &arr[i]);
    }
}
void setMelody(int i){
	if(i==1)
		Change_Melody(harrypotter, harrypotter_size);
	else if(i==2){
		Change_Melody(super_mario_bros, super_mario_bros_size);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
