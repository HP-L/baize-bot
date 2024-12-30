/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.h"
#include "fonts.h"
#include "adc.h"
#include "math3D.h"
#include "fatfs.h"
#include "usart.h"
#include "tim.h"
#include "Fun_Infrared.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern IMU_SensorData_Raw_Structer IMU_SensorData_Raw;
extern uint16_t pwmVal;
extern ADC_HandleTypeDef hadc1;
extern uint16_t ADC_BAT_Value;
extern uint16_t ADC_KEY_Value;
extern uint8_t KEY_Flag;

float B[24]; //三维坐标数组
float C[16]; //投影信息影子数组
float D[16]; //投影信息数组
int16_t j = 0; //角度变量

uint8_t send_Flag = 1;
uint32_t send_Code = 0;
uint32_t receive_Code = 0;
uint8_t receive[33] = { 0 };
uint8_t receive_Flag = 0;
uint8_t addr = 0x00;
uint8_t data = 0xA2;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void SD_Card_Test(void);
void delay_us(uint32_t nus);
void IR_SendByte(uint8_t data);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_UART_Transmit(&huart2, "SD_disk_initialize...\r\n", 23, 0x0100);

//	uint8_t flag_sd_disk = 4;
//	flag_sd_disk = SD_disk_initialize(0);
//	if (flag_sd_disk == STA_NOINIT){
//		HAL_UART_Transmit(&huart2, &flag_sd_disk, 1, 0x0100);
//		HAL_UART_Transmit(&huart2, "\r\n", 2, 0x0100);
//	}else{
//		SD_Card_Test();
//	}
  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
//	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)ADC_Value,2);	 //启动ADC转换
	for (uint8_t i = 0; i < 24; i++) {
		B[i] = Rect[i]; //三维坐标数组赋值
	}
	for (uint8_t i = 0; i < 16; i++) {
		C[i] = D[i]; //影子数组赋初值
	}

	/* Infinite loop */
	for (;;) {

#ifdef DEBUG
		ST7789_WriteString(30 - 11, 40 + 20 * 0, "X", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 11, 40 + 20 * 0, IMU_SensorData_Raw.ACC_X,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 1, "Y", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 11, 40 + 20 * 1, IMU_SensorData_Raw.ACC_Y,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 2, "Z", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 11, 40 + 20 * 2, IMU_SensorData_Raw.ACC_Z,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 3, "GX", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 22, 40 + 20 * 3, IMU_SensorData_Raw.GYR_X,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 4, "GY", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 22, 40 + 20 * 4, IMU_SensorData_Raw.GYR_Y,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 5, "GZ", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 22, 40 + 20 * 5, IMU_SensorData_Raw.GYR_Z,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 6, "T", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 11, 40 + 20 * 6, IMU_SensorData_Raw.Temp,
				Font_11x18, RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 7, "LCD", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 33, 40 + 20 * 7, pwmVal, Font_11x18, RED,
		BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 8, "KEY", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 33, 40 + 20 * 8, KEY_Flag, Font_11x18,
		RED, BLACK);
		ST7789_WriteString(30 - 11, 40 + 20 * 9, "BAT", Font_11x18, RED, BLACK);
		ST7789_DisplayNumber(30 + 33, 40 + 20 * 9, ADC_BAT_Value, Font_11x18,
		RED, BLACK);

		/* 3D */
//		Th_Rotation(j,B,X,24);//对B数组中的三维向量沿X轴旋转j角度，B数组长度：24（8 * 3）
//		Th_Projection(B,D,380,60,16);//将B数组中三维向量投影至D中，视距200，视野60，D数组长度：16（8 * 2）
//		for(uint8_t i = 0;i < 24;i ++)
//		{
//			B[i] = Rect[i];	//重装B数组
//		}
//		Cart_Draw3DRect(D,C,WHITE,1);//绘制D数组中的投影信息，并使用C数组清除上一次绘制痕迹
//		for(uint8_t i = 0;i < 16;i ++)
//		{
//			C[i] = D[i];//将C数组赋值为上一次变换的投影信息
//		}
//		j = j + 6;//角度 + 6
//		if(j >= 360){j =0;}
//		osDelay(10);
		/**/

		LintToLine(20, 60, 220, 60, WHITE, 20, 60, 60, 220);
		LintToLine(180, 200, 180, 300, WHITE, 180, 300, 50, 300);
//		int i = 0;
//		for(i = 0;i<320;i+=2){
//			ST7789_DrawLine(40, i, 80, 80, WHITE);
//		}

#endif
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
		/* 调节屏幕亮度函数 */
		if (KEY_Flag == 1) {
			osDelay(500);
			if (ADC_KEY_Value <= 100) {
				while (ADC_KEY_Value <= 100) {
					pwmVal -= 200;
					if (pwmVal <= 200) {
						pwmVal = 201;
						break;
					}
					osDelay(10);
				}
			}
			if (pwmVal <= 2000) {
				pwmVal = 2001;
			}
			pwmVal -= 1000;
		} else if (KEY_Flag == 2) {
			osDelay(500);
			if (ADC_KEY_Value >= 3900) {
				while (ADC_KEY_Value >= 3900) {
					pwmVal += 200;
					if (pwmVal >= 63000) {
						pwmVal = 62800;
						break;
					}
					osDelay(10);
				}
			}
			if (pwmVal >= 63000) {
				pwmVal = 63000;
			}
			pwmVal += 1000;
		}
		osDelay(1);
	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the myTask03 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	/* Infinite loop */
	for (;;) {
		BSP_MPU6050_UpdateSensors();
//	char buffer[32];
//			while (pwmVal) {
//
//				pwmVal-=100;
////				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pwmVal);   //修改比较值，修改占空比
//				TIM2->CCR1 = pwmVal;     //与上方相同
//				HAL_Delay(1);
//
////				buffer[0] = (pwmVal / 100) / 10 + 48;
////				buffer[1] = (pwmVal / 100) % 10 + 48;
////				buffer[2] = (pwmVal % 100) / 10 + 48;
////				buffer[3] = (pwmVal % 100) % 10 + 48;
//			}

		HAL_ADC_Start(&hadc1);	 //启动ADC转换
		HAL_ADC_PollForConversion(&hadc1, 0xFFFF);   //等待转换完成，50为最大等待时间，单位为ms
		ADC_KEY_Value = HAL_ADC_GetValue(&hadc1);   //获取AD值
		HAL_ADC_Stop(&hadc1);	 //启动ADC转换

		HAL_ADC_Start(&hadc2);	 //启动ADC转换
		HAL_ADC_PollForConversion(&hadc2, 0xFFFF);   //等待转换完成，50为最大等待时间，单位为ms
		ADC_BAT_Value = HAL_ADC_GetValue(&hadc2);   //获取AD值
		HAL_ADC_Stop(&hadc2);	 //启动ADC转换

		if (ADC_KEY_Value <= 100) {
			/* KEY_Flag DOWN */
			KEY_Flag = 1;
		} else if (ADC_KEY_Value >= 3900) {
			/* KEY_Flag UP */
			KEY_Flag = 2;
		} else if (ADC_KEY_Value >= 1500 && ADC_KEY_Value <= 2500) {
			/* KEY_Flag UP */
			KEY_Flag = 0;
		} else if (ADC_KEY_Value >= 500 && ADC_KEY_Value <= 1500) {
			/* KEY_Flag OK */
			KEY_Flag = 3;
		}

		TIM2->CCR1 = pwmVal;

		if (send_Flag == 1) {
			send_Flag = 0;
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
			HAL_UART_Transmit(&huart2, "DELAY:9000\n", 11, 0x0100);
			delay_us(9000);
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
			HAL_UART_Transmit(&huart2, "DELAY:4400\n", 11, 0x0100);
			delay_us(4500);
			HAL_UART_Transmit(&huart2, "IR_SendByte BEGIN\n", 18, 0x0100);
			IR_SendByte(addr);
			uint8_t len = 1;
			for (int i = 0; i < len; i++) {
				IR_SendByte(data);
			}
			send_Flag = 1;
			HAL_UART_Transmit(&huart2, "IR_SendByte END\n", 16, 0x0100);
		}

		osDelay(1);

	}
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void SD_Card_Test(void) {
	FATFS FatFs;
	FIL Fil;
	FRESULT FR_Status;
	FATFS *FS_Ptr;
	UINT RWC, WWC; // Read/Write Word Counter
	DWORD FreeClusters;
	uint32_t TotalSize, FreeSpace;
	char RW_Buffer[200];
	//------------------[ Mount The SD Card ]--------------------
	FR_Status = f_mount(&FatFs, "", 1);
	if (FR_Status != FR_OK) {
		HAL_UART_Transmit(&huart2,
				"Error! While Mounting SD Card, Error Code: (%i)\r\n", 49,
				0x0100);
//    break;
	}
	HAL_UART_Transmit(&huart2, "SD Card Mounted Successfully! \r\n\n", 33,
			0x0100);
	//------------------[ Get & Print The SD Card Size & Free Space ]--------------------
	f_getfree("", &FreeClusters, &FS_Ptr);
	TotalSize = (uint32_t) ((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
	FreeSpace = (uint32_t) (FreeClusters * FS_Ptr->csize * 0.5);
	HAL_UART_Transmit(&huart2, "Total SD Card Size:", 33, 0x0100);
	HAL_UART_Transmit(&huart2, &TotalSize, sizeof(TotalSize), 0x0100);
	HAL_UART_Transmit(&huart2, "\n", 1, 0x0100);
	HAL_UART_Transmit(&huart2, "Free SD Card Space:", 33, 0x0100);
	HAL_UART_Transmit(&huart2, &FreeSpace, sizeof(FreeSpace), 0x0100);
	HAL_UART_Transmit(&huart2, "\n", 1, 0x0100);
}

void IR_SendByte(uint8_t data) {
	uint8_t idata = ~data;
	uint16_t send_code = data << 8 | idata;
	for (int i = 15; i >= 0; i--) {
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		delay_us(560);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
		if ((send_code >> i) & 0x01) //正向
			delay_us(1690);
		else
			delay_us(560);
//		if((send_code>>i) & 0x01)//反向
//			delay_us(560);
//		else
//			delay_us(1690);
	}
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	delay_us(560);
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

void delay_us(uint32_t nus) {
	uint32_t ticks;
	uint32_t told, tnow, reload, tcnt = 0;
	if ((0x0001 & (SysTick->CTRL)) == 0)    //定时器未工作
		vPortSetupTimerInterrupt();  //初始化定时器

	reload = SysTick->LOAD;                     //获取重装载寄存器值
	ticks = nus * (SystemCoreClock / 1000000);  //计数时间值

	vTaskSuspendAll();  //阻止OS调度，防止打断us延时
	told = SysTick->VAL;  //获取当前数值寄存器值（开始时数值）
	while (1) {
		tnow = SysTick->VAL; //获取当前数值寄存器值
		if (tnow != told)  //当前值不等于开始值说明已在计数
				{
			if (tnow < told)  //当前值小于开始数值，说明未计到0
				tcnt += told - tnow; //计数值=开始值-当前值

			else
				//当前值大于开始数值，说明已计到0并重新计数
				tcnt += reload - tnow + told;   //计数值=重装载值-当前值+开始值  （
												//已从开始值计到0）

			told = tnow;   //更新开始值
			if (tcnt >= ticks)
				break;  //时间超过/等于要延迟的时间,则退出.
		}
	}
	xTaskResumeAll();	//恢复OS调度
}
//SystemCoreClock为系统时钟(system_stmf4xx.c中)，通常选择该时钟作为
//systick定时器时钟，根据具体情况更改

/* USER CODE END Application */

