/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
 * Links
 *  1. Json: https://shequ.stmicroelectronics.cn/thread-634459-1-1.html
 *  2. << 16 : https://blog.csdn.net/m0_46577050/article/details/127897059
 *  3. ws2812b : https://github.com/Lzx-James/WS2812_STM32F103C8T6/blob/main/Hardware/WS2812.c
 *  4. pwm : https://blog.csdn.net/as480133937/article/details/99231677
 *  5. st7735 : https://github.com/ScarsFun/STM32F103C8T6_cubeIDE_ST7735_1.8_DMA
 *  6. c_class: https://blog.csdn.net/weixin_42073412/article/details/100586354
 *  7. IR_NEC: https://blog.csdn.net/u011657059/article/details/107215673
 *  8. TF_card: https://deepbluembedded.com/stm32-sd-card-spi-fatfs-tutorial-examples
 *  9. ST7789: 改编自https://github.com/lbthomsen/stm32-st7789
 *  10. MPU6050: https://blog.csdn.net/WilliamNUAA/article/details/119673424
 *  11. FreeRTOS: https://shequ.stmicroelectronics.cn/thread-639646-1-1.html
 *  12. ADC|DMA: https://blog.csdn.net/tangxianyu/article/details/121149981
 *  13. freertos msdelay: https://blog.csdn.net/w237838/article/details/134771598
 *  14. HAL-IR_O: https://blog.csdn.net/m0_69414528/article/details/134275576
 *
 * 问题记录
 *  1. DMA 和外部中断同时开启使用时会出现显示异常的问题（已解决
 *  	原因：可能是中断中的中断导致出问题，修改成外部中断触发接收串口，但显示函数在主循环中问题解决
 *  2. 同一个ADC不同通道识别间隔过短会相互影响数值，对于响应要求较快的场景不适用
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//用户添加头文
#include "cJSON.h"
#include "st7789.h"
#include "fonts.h"
#include "testimg.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//添加 typedef 区域
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//添加#define 区域
/* choose whether use DEBUG or not */
#define DEBUG

#define I2C_MEMADD_SIZE_8BIT 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//添加宏区
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//添加变量区域
uint8_t mpu6050_value = 0;
uint8_t r = 0;   //屏幕的显示角
uint8_t CS = 0;  // 第几块屏幕亮

uint8_t USART_RX_ONE_LINE_LEN[10];      //循环接收数组
uint8_t USART_RX_ONE_LINE[10];      //循环接收数组
char USART_RX_LINE[2800 + 2];      //循环处理接收数组
uint8_t USART_RX_LINE_L[2800 + 2];      //除去AAAAABBBBB标识符的数据
uint8_t DataBuff[5000]; //保存接收到的数据的数
uint32_t RxLine = 0;           //接收到的数据长度
uint32_t pin_pb6_value;
char test[1] = "1";

uint32_t pwmVal = 60000;   //PWM占空
//ADC部分初始化
uint16_t ADC_BAT_Value;
uint16_t ADC_KEY_Value;
uint8_t KEY_Flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//用户添加函数区域
//void Move_coordinate(void){
//
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void json_parse(char *json_string) {

//	cJSON *parsedJson = cJSON_Parse(json_string);
//	if (!parsedJson) {
//		char error[] = "Error parsing JSON\n";
//		HAL_UART_Transmit(&huart1, (uint8_t*) error, sizeof(error) - 1, 100);
//		return;
//	}
//
//	cJSON *light_value = cJSON_GetObjectItemCaseSensitive(parsedJson,
//			"light_value");
//	if (light_value) {
//		pwmVal = light_value->valueint;
//	}
//
//	cJSON *cpu = cJSON_GetObjectItemCaseSensitive(parsedJson, "cpu");
//	if (cpu) {
//		cJSON *used = cJSON_GetObjectItemCaseSensitive(cpu, "used");
//		if (used) {
//			char buffer[32];
//			memcpy(buffer, &(used->valuedouble), sizeof(double));
//			cjsonCpuUsed = used->valuedouble;
//			myCoordinate.cpuY13 = myCoordinate.cpuY12;
//			myCoordinate.cpuY12 = myCoordinate.cpuY11;
//			myCoordinate.cpuY11 = myCoordinate.cpuY10;
//			myCoordinate.cpuY10 = myCoordinate.cpuY9;
//			myCoordinate.cpuY9 = myCoordinate.cpuY8;
//			myCoordinate.cpuY8 = myCoordinate.cpuY7;
//			myCoordinate.cpuY7 = myCoordinate.cpuY6;
//			myCoordinate.cpuY6 = myCoordinate.cpuY5;
//			myCoordinate.cpuY5 = myCoordinate.cpuY4;
//			myCoordinate.cpuY4 = myCoordinate.cpuY3;
//			myCoordinate.cpuY3 = myCoordinate.cpuY2;
//			myCoordinate.cpuY2 = myCoordinate.cpuY1;
//			myCoordinate.cpuY1 = cjsonCpuUsed*10;
//			clean_lcd_flag = 1;
//
//			HAL_UART_Transmit(&huart1, buffer, strlen(buffer), 100);
//		}
//
//		cJSON *used_list = cJSON_GetObjectItemCaseSensitive(cpu, "used_list");
//		if (used_list) {
//			int i;
//			for (i = 0; i < cJSON_GetArraySize(used_list); i++) {
//				cJSON *item = cJSON_GetArrayItem(used_list, i);
//				if (item) {
//					char buffer[32];
//					memcpy(buffer, &(item->valuedouble), sizeof(double));
//					HAL_UART_Transmit(&huart1, (uint8_t*) buffer,
//							strlen(buffer), 100);
//				}
//			}
//		}
//
//		cJSON *cpuCount = cJSON_GetObjectItemCaseSensitive(cpu, "cpuCount");
//		if (cpuCount) {
//			char buffer[32];
//			cjsonCpuCount = cpuCount->valueint;
//			memcpy(buffer, &(cpuCount->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *cpuName = cJSON_GetObjectItemCaseSensitive(cpu, "cpuName");
//		if (cpuName) {
//			char buffer[32];
//			for(int i = 0;i < 40;i++){
//				cjsonCpuName[i] = ' ';
//			}
//
//			strcpy(cjsonCpuName, cpuName->valuestring);
//			HAL_UART_Transmit(&huart1, (uint8_t*) cpuName->valuestring,
//					strlen(cpuName->valuestring), 100);
//		}
//
//		cJSON *cpuCore = cJSON_GetObjectItemCaseSensitive(cpu, "cpuCore");
//		if (cpuCore) {
//			char buffer[32];
//			cjsonCpuCore = cpuCore->valueint;
//			memcpy(buffer, &(cpuCore->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *cpuThreads = cJSON_GetObjectItemCaseSensitive(cpu,
//				"cpuThreads");
//		if (cpuThreads) {
//			char buffer[32];
//			cjsonCpuThreads = cpuThreads->valueint;
//			memcpy(buffer, &(cpuThreads->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//	}
//
//	cJSON *load = cJSON_GetObjectItemCaseSensitive(parsedJson, "load");
//	if (load) {
//		cJSON *one = cJSON_GetObjectItemCaseSensitive(load, "one");
//		if (one) {
//			char buffer[32];
//			cjson_load_one = one->valuedouble;
//			memcpy(buffer, &(one->valuedouble), sizeof(double));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *five = cJSON_GetObjectItemCaseSensitive(load, "five");
//		if (five) {
//			char buffer[32];
//			cjson_load_five = five->valuedouble;
//			memcpy(buffer, &(five->valuedouble), sizeof(double));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *fifteen = cJSON_GetObjectItemCaseSensitive(load, "fifteen");
//		if (fifteen) {
//			char buffer[32];
//			cjson_load_fifteen = fifteen->valuedouble;
//			memcpy(buffer, &(fifteen->valuedouble), sizeof(double));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *max = cJSON_GetObjectItemCaseSensitive(load, "max");
//		if (max) {
//			char buffer[32];
//			cjson_load_max = max->valuedouble;
//			memcpy(buffer, &(max->valuedouble), sizeof(double));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *limit = cJSON_GetObjectItemCaseSensitive(load, "limit");
//		if (limit) {
//			char buffer[32];
//			cjson_load_limit = limit->valueint;
//			memcpy(buffer, &(limit->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *safe = cJSON_GetObjectItemCaseSensitive(load, "safe");
//		if (safe) {
//			char buffer[32];
//			cjson_load_safe = safe->valuedouble;
//			memcpy(buffer, &(safe->valuedouble), sizeof(double));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//	}
//
//	cJSON *mem = cJSON_GetObjectItemCaseSensitive(parsedJson, "mem");
//	if (mem) {
//		cJSON *memTotal = cJSON_GetObjectItemCaseSensitive(mem, "memTotal");
//		if (memTotal) {
//			char buffer[32];
//			cjson_mem_total = memTotal->valueint;
//			memcpy(buffer, &(memTotal->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *memFree = cJSON_GetObjectItemCaseSensitive(mem, "memFree");
//		if (memFree) {
//			char buffer[32];
//			cjson_mem_free = memFree->valueint;
//			memcpy(buffer, &(memFree->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *memUsed = cJSON_GetObjectItemCaseSensitive(mem, "memUsed");
//		if (memUsed) {
//			char buffer[32];
//			cjsonMemUsed = memUsed->valueint;
//			memcpy(buffer, &(memUsed->valueint), sizeof(int));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *memUsedPercent = cJSON_GetObjectItemCaseSensitive(mem,
//				"memUsedPercent");
//		if (memUsedPercent) {
//			char buffer[32];
//			memcpy(buffer, &(memUsedPercent->valuedouble), sizeof(double));
////			snprintf(buffer, sizeof(buffer), "Memory Used Percent: %.2f%%\n",
////					memUsedPercent->valuedouble);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//	}
//
//	cJSON *disk = cJSON_GetObjectItemCaseSensitive(parsedJson, "disk");
//	if (disk) {
//		int i;
//		for (i = 0; i < cJSON_GetArraySize(disk); i++) {
//			cJSON *item = cJSON_GetArrayItem(disk, i);
//			if (item) {
//				cJSON *path = cJSON_GetObjectItemCaseSensitive(item, "path");
//				if (path) {
//					char buffer[32];
////					memcpy(buffer, &(path->valuestring), sizeof(string));
////					snprintf(buffer, sizeof(buffer), "Disk Path[%d]: %s\n", i,
////							path->valuestring);
//					HAL_UART_Transmit(&huart1, (uint8_t*) path->valuestring,
//							strlen(path->valuestring), 100);
//				}
//
//				cJSON *size = cJSON_GetObjectItemCaseSensitive(item, "size");
//				if (size) {
//					int j;
//					for (j = 0; j < cJSON_GetArraySize(size); j++) {
//						cJSON *size_item = cJSON_GetArrayItem(size, j);
//						if (size_item) {
//							char buffer[32];
////							snprintf(buffer, sizeof(buffer),
////									"Disk Size[%d][%d]: %s\n", i, j,
////									size_item->valuestring);
//							HAL_UART_Transmit(&huart1,
//									(uint8_t*) size_item->valuestring,
//									strlen(size_item->valuestring), 100);
//						}
//					}
//				}
//
//				cJSON *inodes = cJSON_GetObjectItemCaseSensitive(item,
//						"inodes");
//				if (inodes) {
//					int j;
//					for (j = 0; j < cJSON_GetArraySize(inodes); j++) {
//						cJSON *inodes_item = cJSON_GetArrayItem(inodes, j);
//						if (inodes_item) {
//							char buffer[32];
////							snprintf(buffer, sizeof(buffer),
////									"Disk Inodes[%d][%d]: %s\n", i, j,
////									inodes_item->valuestring);
//							HAL_UART_Transmit(&huart1,
//									(uint8_t*) inodes_item->valuestring,
//									strlen(inodes_item->valuestring), 100);
//						}
//					}
//				}
//			}
//		}
//	}
//
//	cJSON *network = cJSON_GetObjectItemCaseSensitive(parsedJson, "network");
//	if (network) {
//		cJSON *up = cJSON_GetObjectItemCaseSensitive(network, "up");
//		if (up) {
//			char buffer[32];
////			snprintf(buffer, sizeof(buffer), "Network Up: %.2f\n",
////					up->valuedouble);
//			memcpy(buffer, &(up->valuedouble), sizeof(double));
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *down = cJSON_GetObjectItemCaseSensitive(network, "down");
//		if (down) {
//			char buffer[32];
//			memcpy(buffer, &(down->valuedouble), sizeof(double));
////			snprintf(buffer, sizeof(buffer), "Network Down: %.2f\n",
////					down->valuedouble);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *upTotal = cJSON_GetObjectItemCaseSensitive(network, "upTotal");
//		if (upTotal) {
//			char buffer[32];
//			memcpy(buffer, &(upTotal->valueint), sizeof(int));
////			snprintf(buffer, sizeof(buffer), "Network Up Total: %d\n",
////					upTotal->valueint);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *downTotal = cJSON_GetObjectItemCaseSensitive(network,
//				"downTotal");
//		if (downTotal) {
//			char buffer[32];
//			memcpy(buffer, &(downTotal->valueint), sizeof(int));
////			snprintf(buffer, sizeof(buffer), "Network Down Total: %d\n",
////					downTotal->valueint);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *downPackets = cJSON_GetObjectItemCaseSensitive(network,
//				"downPackets");
//		if (downPackets) {
//			char buffer[32];
//			memcpy(buffer, &(downPackets->valueint), sizeof(int));
////			snprintf(buffer, sizeof(buffer), "Network Down Packets: %d\n",
////					downPackets->valueint);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *upPackets = cJSON_GetObjectItemCaseSensitive(network,
//				"upPackets");
//		if (upPackets) {
//			char buffer[32];
//			memcpy(buffer, &(upPackets->valueint), sizeof(int));
////			snprintf(buffer, sizeof(buffer), "Network Up Packets: %d\n",
////					upPackets->valueint);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//	}
//
//	cJSON *io = cJSON_GetObjectItemCaseSensitive(parsedJson, "io");
//	if (io) {
//		cJSON *write = cJSON_GetObjectItemCaseSensitive(io, "write");
//		if (write) {
//			char buffer[32];
//			memcpy(buffer, &(write->valueint), sizeof(int));
////			snprintf(buffer, sizeof(buffer), "IO Write: %d\n", write->valueint);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *read = cJSON_GetObjectItemCaseSensitive(io, "read");
//		if (read) {
//			char buffer[32];
//			memcpy(buffer, &(read->valueint), sizeof(int));
////			snprintf(buffer, sizeof(buffer), "IO Read: %d\n", read->valueint);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//	}
//
//	cJSON *boot = cJSON_GetObjectItemCaseSensitive(parsedJson, "boot");
//	if (boot) {
//		cJSON *timestamp = cJSON_GetObjectItemCaseSensitive(boot, "timestamp");
//		if (timestamp) {
//			char buffer[32];
//			memcpy(buffer, &(timestamp->valuedouble), sizeof(double));
////			snprintf(buffer, sizeof(buffer), "Boot Timestamp: %.2f\n",
////					timestamp->valuedouble);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *runtime = cJSON_GetObjectItemCaseSensitive(boot, "runtime");
//		if (runtime) {
//			char buffer[32];
//			memcpy(buffer, &(runtime->valuedouble), sizeof(double));
////			snprintf(buffer, sizeof(buffer), "Boot Runtime: %.2f\n",
////					runtime->valuedouble);
//			HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//		}
//
//		cJSON *datetime = cJSON_GetObjectItemCaseSensitive(boot, "datetime");
//		if (datetime) {
//			char buffer[32];
//			strcpy(cjsonBootDatetime, datetime->valuestring);
//			HAL_UART_Transmit(&huart1, (uint8_t*) datetime->valuestring,
//					strlen(datetime->valuestring), 100);
//		}
//	}
//
//	cJSON *time = cJSON_GetObjectItemCaseSensitive(parsedJson, "time");
//	if (time) {
//		char buffer[32];
//		memcpy(buffer, &(time->valuedouble), sizeof(double));
////		snprintf(buffer, sizeof(buffer), "Time: %.2f\n", time->valuedouble);
//		HAL_UART_Transmit(&huart1, (uint8_t*) buffer, strlen(buffer), 100);
//	}
//
//	// 释放解析后的JSON对象
//	cJSON_Delete(parsedJson);
}
//
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_FATFS_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

	/* 串口 1debug */
//	HAL_UART_Transmit(&huart1, "INIT OK\n", 8, 0xffff);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	/* 锁住 */
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(EN_8266_GPIO_Port, EN_8266_Pin, GPIO_PIN_RESET);

	/* ADC校准 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	/* 串口 1debug */
//	HAL_UART_Transmit(&huart1, "wait 8266\n", 10, 0xffff);
//	while (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)) {
//		HAL_Delay(50);
//		/* 串口 1debug */
//		HAL_UART_Transmit(&huart1, "wait 8266\n", 10, 0xffff);
//	}
	/* 串口 1debug */
//	HAL_UART_Transmit(&huart1, "8266 OK\n", 8, 0xffff);
	/* 串口中断开启接收长度 1 */
//	HAL_UART_Receive_IT(&huart2, &USART_RX_ONE_LINE_LEN, 1);
	/* 解析json */
//	json_parse(&USART_RX_LINE);
	/* 设置背光 */
	pwmVal = 40000;
	/* 调背光 1 -> 3V3 -> 0V | 1400 -> 0.01V -> 3V3 */
	TIM2->CCR1 = pwmVal;
	/* 初始化显示屏 */
	ST7789_Init();
	ST7789_WriteString(30, 30, "MPU6050_Init...", Font_11x18, RED, BLACK);
	BSP_MPU6050_Init();
	ST7789_Fill_Color(BLACK);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
//		HAL_I2C_Mem_Read(&hi2c2, 0xD1, 0x75, I2C_MEMADD_SIZE_8BIT, &mpu6050_value, 1, 0xfff);
//		ST7789_Test();
//		BSP_MPU6050_UpdateSensors();
//		ST7789_WriteString(30, 40+20*7, " !\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~",Font_11x18, RED, BLACK);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/****************************************************************************
 * 函数名称: HAL_UART_RxCpltCallback()
 * 功能: 串口中断函数
 * 参数: None
 * 全局变量 ALL
 *  : 
 */

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart->Instance == USART2) // USART1
//	{
//		/* 串口 1debug */
//		HAL_UART_Transmit(&huart1, "IN UART_RxCpltCallback\n", 23, 100);
//		/*判断 PB6 为上升或下降沿触*/
//		pin_pb6_value = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
//		/* 高电 接收数据 */
//		if (pin_pb6_value == 1) {
//			/* 回复8266，GPIO拉底 解锁 已经完成第一字节串口接收 */
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//			/* 串口 1debug */
////			HAL_UART_Transmit(&huart1, &USART_RX_LINE,
////					atoi(USART_RX_ONE_LINE) + 1, 100);
////			CS++;
////			if (CS > 4) {
////				CS = 1;
////			}
//			/* 串口中断启接收长 1 */
//			HAL_UART_Receive_IT(&huart2, &USART_RX_ONE_LINE_LEN, 1);
//			/* 解析json */
//			json_parse(&USART_RX_LINE);
//		}
//		/* 低电 接收长度 */
//		if (pin_pb6_value == 0) {
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, "pin_pb6_value == 0\n", 19, 100);
//			/* 回复8266，GPIO拉底 解锁 已经完成第一字节串口接收 */
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, "GPIO7 DOWN\n", 11, 100);
//			/* 串口 2 接收长度2 */
//			HAL_UART_Receive(&huart2, &USART_RX_ONE_LINE,
//					atoi(USART_RX_ONE_LINE_LEN) + 2, 0xFFFF);
//			/* 回复8266，GPIO拉高 已经完成第二字节串口接收 */
//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, "GPIO7 UP\n", 9, 100);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, "\n############################\n", 30,
//					100);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, &USART_RX_ONE_LINE_LEN, 1, 100);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, "\n############################\n", 30,
//					100);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, &USART_RX_ONE_LINE,
//					atoi(USART_RX_ONE_LINE_LEN) + 2, 100);
//			/* 串口 1debug */
//			HAL_UART_Transmit(&huart1, "\n############################\n", 30,
//					100);
//			/* 启下次接收数据内容中 */
//			HAL_UART_Receive_IT(&huart2, &USART_RX_LINE,
//					atoi(USART_RX_ONE_LINE) + 2);
//		}
//	}
//}

/* USER CODE END 4 */

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
	while (1) {
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
