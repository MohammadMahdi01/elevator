/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef enum {down=-1, fixed,up} direction;
typedef enum {false, true} bool;
typedef struct
{
    uint16_t frequency;
    uint16_t duration;
} Tone;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ARRAY_LENGTH(arr) (sizeof(arr) / sizeof((arr)[0]))

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool p=true;
extern TIM_HandleTypeDef htim1;
extern int curLevel;
extern int newDest;
extern int curDest;
extern int alarmed;
extern char input[20];
extern direction dir;
TIM_HandleTypeDef *pwm_timer = &htim1;	// Point to PWM Timer configured in CubeMX
uint32_t pwm_channel = TIM_CHANNEL_1;   // Select configured PWM channel number

const Tone *volatile melody_ptr;
volatile uint16_t melody_tone_count;
volatile uint16_t current_tone_number;
volatile uint32_t current_tone_end;
volatile uint16_t volume = 1;          // (0 - 1000)
volatile uint32_t last_button_press;
/*
 *
 8 352
 4 705
 2 1411
 -8 529
 -4 1058
 -2 2117






 */
const Tone harrypotter[] = {

	{REST,  0},
	{NOTE_D4,705}, // E7 x2
	{NOTE_G4,1058}, // x3 <-- Silence
	{NOTE_AS4,352}, // E7
	{NOTE_A4,705}, // x3
	{NOTE_G4,1411}, // C7
	{NOTE_D5,705}, // E7
	{NOTE_C5,2117}, // x3
	{NOTE_A4,2117}, // G7
	{NOTE_G4,1058}, // x3
	{NOTE_AS4,352}, // G6
	{NOTE_A4,705}, // x3
	{NOTE_F4,1411},
	{NOTE_GS4,705}, // E7 x2
	{NOTE_D4,4234}, // x3 <-- Silence
	{NOTE_D4,705}, // E7
	{NOTE_G4,1058}, // x3
	{NOTE_AS4,352}, // C7
	{NOTE_A4,705}, // E7
	{NOTE_G4,1411}, // x3
	{NOTE_D5,705}, // G7
	{NOTE_F5,1411}, // x3
	{NOTE_E5,705}, // G6
	{NOTE_DS5,1411}, // x3
	{NOTE_B4,705}, // C7
	{NOTE_DS5,1058}, // x2
	{NOTE_D5,352}, // G6
	{NOTE_CS5,705}, // x2
	{NOTE_CS4,1411}, // E6
	{NOTE_B4,705}, // x2
	{NOTE_G4,4234}, // A6
	{NOTE_AS4,705}, // x1
	{NOTE_D5,1411}, // B6
	{NOTE_AS4,705}, // x1
	{NOTE_D5,1411}, // AS6
	{NOTE_AS4,705}, // A6
	{NOTE_DS5,1411}, // x1
	{NOTE_CS5,1411}, // G6
	{NOTE_A4,705}, // E7
	{NOTE_AS4,1058}, // G7
	{NOTE_D5,352}, // A7
	{NOTE_CS5,705}, // x1
	{NOTE_CS4,2117}, // F7
	{NOTE_D4,705}, // G7
	{NOTE_D5,4234}, // x1
	{REST,705}, // E7
	{NOTE_AS4,705}, // x1
	{NOTE_D5,1411}, // C7
	{NOTE_AS4,705}, // D7
	{NOTE_D5,1411}, // B6
	{NOTE_AS4,705}, // x2

	{NOTE_F5,1411}, // C7
	{NOTE_E5,705}, // x2
	{NOTE_DS5,1411}, // G6
	{NOTE_B4,705}, // x2
	{NOTE_DS5,1058}, // E6
	{NOTE_D5,306}, // x2
	{NOTE_CS5,705}, // A6
	{NOTE_CS4,1411}, // x1
	{NOTE_AS4,705}, // B6
	{NOTE_G4,4234}, // x1

	{   0,  0}	// <-- Disable PWM
};

const Tone super_mario_bros[] = {
	{2637,306}, // E7 x2
	{   0,153}, // x3 <-- Silence
	{2637,153}, // E7
	{   0,153}, // x3
	{2093,153}, // C7
	{2637,153}, // E7
	{   0,153}, // x3
	{3136,153}, // G7
	{   0,459}, // x3
	{1586,153}, // G6
	{   0,459}, // x3

	{2093,153}, // C7
	{   0,306}, // x2
	{1586,153}, // G6
	{   0,306}, // x2
	{1319,153}, // E6
	{   0,306}, // x2
	{1760,153}, // A6
	{   0,153}, // x1
	{1976,153}, // B6
	{   0,153}, // x1
	{1865,153}, // AS6
	{1760,153}, // A6
	{   0,153}, // x1

	{1586,204}, // G6
	{2637,204}, // E7
	{3136,204}, // G7
	{3520,153}, // A7
	{   0,153}, // x1
	{2794,153}, // F7
	{3136,153}, // G7
	{   0,153}, // x1
	{2637,153}, // E7
	{   0,153}, // x1
	{2093,153}, // C7
	{2349,153}, // D7
	{1976,153}, // B6
	{   0,306}, // x2

	{2093,153}, // C7
	{   0,306}, // x2
	{1586,153}, // G6
	{   0,306}, // x2
	{1319,153}, // E6
	{   0,306}, // x2
	{1760,153}, // A6
	{   0,153}, // x1
	{1976,153}, // B6
	{   0,153}, // x1
	{1865,153}, // AS6
	{1760,153}, // A6
	{   0,153}, // x1

	{1586,204}, // G6
	{2637,204}, // E7
	{3136,204}, // G7
	{3520,153}, // A7
	{   0,153}, // x1
	{2794,153}, // F7
	{3136,153}, // G7
	{   0,153}, // x1
	{2637,153}, // E7
	{   0,153}, // x1
	{2093,153}, // C7
	{2349,153}, // D7
	{1976,153}, // B6

	{   0,  0}	// <-- Disable PWM
};
int super_mario_bros_size= ARRAY_LENGTH(super_mario_bros);
int harrypotter_size= ARRAY_LENGTH(harrypotter);
//Tone currentMelody[]=super_mario_bros;

void PWM_Start()
{
    HAL_TIM_PWM_Start(pwm_timer, pwm_channel);
}
void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
{
    if (pwm_freq == 0 || pwm_freq > 20000)
    {
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, 0);
    }
    else
    {
        const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
        const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
        const uint32_t timer_clock = internal_clock_freq / prescaler;
        const uint32_t period_cycles = timer_clock / pwm_freq;
        const uint32_t pulse_width = volume * period_cycles / 1000 / 2;

        pwm_timer->Instance->PSC = prescaler - 1;
        pwm_timer->Instance->ARR = period_cycles - 1;
        pwm_timer->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
    }
}
void Change_Melody(const Tone *melody, uint16_t tone_count)
{
    melody_ptr = melody;
    melody_tone_count = tone_count;
    current_tone_number = 0;
}

void Update_Melody()
{
    if ((HAL_GetTick() > current_tone_end))
    {
        const Tone active_tone = *(melody_ptr + current_tone_number%melody_tone_count);
        PWM_Change_Tone(active_tone.frequency, volume);
        current_tone_end = HAL_GetTick() + active_tone.duration;
        if(p)
        current_tone_number++;
    }
}
void numberToBCD(int i){
    int x1 = i&1;
    int x2 = i&2;
    int x3 = i&4;
    int x4 = i&8;
    if(x1>0)
      {x1=1;}
    if(x2>0)
      {x2=1;}
    if(x3>0)
      {x3=1;}
    if(x4>0)
      {x4=1;}

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, x1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, x2);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, x3);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, x4);
}
void direct(int i){
	if(i==2){
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<6, 1);
		if(dir<0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<5, 1);

		}
		else if(dir==0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<3, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<5, 0);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<1, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<5, 0);
	}
	}
	else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<6, 1);
		if(dir<0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<1, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<4, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<5, 0);

		}
		else if(dir==0){
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<2, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<3, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<5, 0);
		}
		else{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<1, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<2, 1);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<3, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<4, 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<5, 1);
		}
	}
}
void showSS(int i){
	if(i==1)
		numberToBCD(newDest);
	else if(i==2){
		numberToBCD(8);
		direct(2);
	}
	else if(i==3)
		direct(3);
	else{
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8<<0|GPIO_PIN_8<<1|GPIO_PIN_8<<2|GPIO_PIN_8<<3|GPIO_PIN_8<<4|GPIO_PIN_8<<5|GPIO_PIN_8<<6, 1);
		numberToBCD(curLevel);
	}

}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  //Update_Melody();
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 and Touch Sense controller.
  */
void EXTI2_TSC_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */

  /* USER CODE END EXTI2_TSC_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */

  /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */

  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
	  HAL_UART_Receive_IT(&huart1, input, sizeof(input));

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
