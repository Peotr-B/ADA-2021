/* USER CODE BEGIN Header */
/**
 * 7мар21
 * Biriuk
 * NUCLEO-L452RE-P
 *
 * Nucleo_ADA-2021
 *
 * STM32CubeIDE Начинаем работать некоторые нюансы использоваиия среды разработки
 * https://www.youtube.com/watch?v=u8zvOeuOdW4&list=RDCMUC2vcuP7iWaX0MVG6hJ-cOkg&start_radio=1#t=0&pbj=1
 *
 * ADC HAL stm32
 * https://istarik.ru/blog/stm32/113.html
 *
 * STM32. АЦП на практике. DMA, прерывания. Переходи с Arduino на STM32
 * https://www.youtube.com/watch?v=4DPMhs-hNMU
 *
 * STM32Cube ADC+PWM регулировка яркости светодиодов используя Ш�?М,АЦП и stm32f4 discovery
 * https://www.youtube.com/watch?v=RiZO9HGM-OY
 *
 * STM32Cube ADC настройка АЦП для регулярного канала STM32F407 discovery
 * https://www.youtube.com/watch?v=Pvw7AeUAYNQ&list=PL7oty_7pMddnTOH7Gpc7mokh96jt-ynVT&index=54
 *
 *STM32CubeIDE. Принципы работы и настройка интерфейса SWO
 *https://www.youtube.com/watch?v=nE-YrKpWjso&list=PL9lkEHy8EJU8a_bqiJXwGTo-uM_cPa98P
 *
    � Программирование МК STM32. УРОК 18. HAL. ADC. Regular Channel. DMA
 *https://www.youtube.com/watch?v=0fpdNWFnggQ&t=279s
 *https://narodstream.ru/stm-urok-18-hal-adc-regular-channel-dma/
 *
 * Урок 4:  STM Studio
 * https://www.rvrobotics.ru/stm32_lesson_4
 *
 * STM32CubeIDE. Установка, настройка и отладка
 * https://www.youtube.com/watch?v=6xhzoDGi4qA
 *
 * Таймеры stm32 HAL - часть первая
 * https://istarik.ru/blog/stm32/118.html
 * D:\YandexDisk\Документы\Biriuk\Soft\STM32\STM32_Материал\STM32L452RE_Документация\STM32L452__Datascheet, стр.39
 *
 * D:\YandexDisk\Документы\Biriuk\Soft\STM32\STM32Cube_Примеры\STM32Cube_FW_L4_V1.15.0\Projects\NUCLEO-L412RB-P\
 * Examples\ADC\ADC_Sequencer\Src\main.c
 *
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "string.h" // это для функции strlen()
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//�?змерение напряжения питания микроконтроллера STM32
//https://microtechnics.ru/izmerenie-napryazheniya-pitaniya-mikrokontrollera-stm32/
#define ADC_REFERENCE_VOLTAGE	1.212	//internal reference voltage - datascheet, s.101
#define ADC_MAX		0xFFF		//Максимальное напряжение питания АЦП в кодах = 4095

// D:\YandexDisk\Документы\Biriuk\Soft\STM32\STM32Cube_Примеры\STM32Cube_FW_L4_V1.15.0\Projects\NUCLEO-L412RB-P\Examples\ADC\ADC_Sequencer\Src\main.c

/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300U)

/* Definitions for DAC parameters */
#define RANGE_12BITS                     (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B))    /* Max digital value with a full range of 12 bits */

/* Definitions of data related to this example */
/* Definition of ADCx conversions data table size */
/* Size of array set to ADC sequencer number of ranks converted,            */
/* to have a rank in each array address.                                    */
#define ADC_CONVERTED_DATA_BUFFER_SIZE   (   4U)

#define MSB_      4096	//��� RANGE_12BITS 2^12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//uint8_t i = 0;
uint32_t  timme = 0;
//float u[4] = {0,};

//float u[ADC_CONVERTED_DATA_BUFFER_SIZE] = {0,};

char trans_str[64] = {0,};
//volatile uint16_t adc[4] = {0,};

volatile uint16_t adc[ADC_CONVERTED_DATA_BUFFER_SIZE] = {0,};

volatile uint8_t flag = 0;
int LED_State = 0;
volatile unsigned long T;
//int _L = 0;

//int SYGNaL_single_mV = 0;	//Напряжение входа АЦП (Uвх)  в мВ
//int SYGNaL_single_mV_hal = 0;	//Напряжение входа АЦП (Uвх)  в мВ  в библиотеке HAL
float SYGNaL_dif_mV = 0;		//Напряжение дифференциального входа АЦП (Uвх.диф)  в мВ
//float SYGNaL_dif_mV_ = 0;
int Temp_Sens = 0;			//Значение температуры кристалла (Temperature Sensor Channel) в град.С х 10
//float Temp_DEG_C = 0;		//Приведенное значение температуры кристалла в град.С
int VDD_mV = 0;				//Значение напряжения питания кристалла (VDD) в мВ
float VDD_V = 0;			//Приведенное значение напряжения кристалла (VDD) в В
//int Vbat_mV = 0;			//Значение напряжения батареи (Vbat Channel) в мВ
float Vbat_V = 0;			//Приведенное значение напряжения батареи (Vbat Channel) в В
//float V_RFINT = 1.212;	//internal reference voltage - datascheet, s.101
//int mcuVoltage_mV = 0;		//Значение напряжения питания в мВ
float mcuVoltage_V = 0;		//Приведенное значение напряжения питания в В
//uint16_t Vrefint_mV = 0;	//�?змеряемое напряжение Vrefint (Vrefint Channel) в мВ
float Vref_V = 0;		//�?змеряемое напряжение Vrefint (Vrefint Channel) в В


volatile uint8_t REPER = 0;
volatile int REPER2 = 0;

//volatile uint16_t ADC_Data[4];

// D:\YandexDisk\Документы\Biriuk\Soft\STM32\STM32Cube_Примеры\STM32Cube_FW_L4_V1.15.0\Projects\NUCLEO-L412RB-P\Examples\ADC\ADC_Sequencer\Src\main.c

/* Peripherals handlers declaration */
/* ADC handler declaration */
//ADC_HandleTypeDef    AdcHandle;


/* Variable containing ADC conversions results */
//N.B. __IO <=> volatile!
//__IO uint16_t aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];	//<=> adc

/* Variables for ADC conversion data computation to physical values */
//__IO uint16_t uhADCxConvertedData_VoltageGPIO_mVolt = 0U;        /* <=> SYGNaL     Value of voltage on GPIO pin (on which is mapped ADC channel) calculated from ADC conversion data (unit: mV) */
//__IO uint16_t uhADCxConvertedData_VrefInt_mVolt = 0U;            /* <=> Vrefint    Value of internal voltage reference VrefInt calculated from ADC conversion data (unit: mV) */
//__IO  int16_t hADCxConvertedData_Temperature_DegreeCelsius = 0U; /* <=> Temp_Sens  Value of temperature calculated from ADC conversion data (unit: degree Celcius) */
//__IO uint16_t uhADCxConvertedData_VrefAnalog_mVolt = 0U;         /* <=> Vbat       Value of analog reference voltage (Vref+), connected to analog voltage supply Vdda, calculated from ADC conversion data (unit: mV) */

/* Variables to manage push button on board: interface between ExtLine interruption and main program */
//uint8_t       ubUserButtonClickCount = 0;      /* Count number of clicks: Incremented after User Button interrupt */
//__IO uint8_t  ubUserButtonClickEvent = RESET;  /* Event detection: Set after User Button interrupt */

/* Variable to report ADC sequencer status */
//uint8_t       ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	//REPER++;
	if (hadc->Instance == ADC1) //check if the interrupt comes from ACD1
	{
		flag = 1;
		//REPER2 = REPER2 + 2;

		//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
		//LED_State = HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin);

		//for (uint8_t j = 0; j < 4; j++)
		//{
			//HAL_ADC_Stop_DMA(&hadc1); // <a> необязательно
			//snprintf(trans_str, 63, "ADC %d\n", adc[j]);
			//snprintf(trans_str, 63, "ADC %d \n", adc);
			//HAL_UART_Transmit(&huart2, (uint8_t*) trans_str, strlen(trans_str), 1000);
			//adc[j] = 0;
			//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc, j);
			//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc, 1);

			//printf("АЦП = %d\n", adc[j]);
			//puts("АЦП работает правильно!!");

			//printf("ADC = %d\n", adc[j]);
			//puts("ADC arbeit richtig");

			//REPER2 ++;
			//REPER2 = 10;

		//}
		//flag = 0;

		//REPER2 = REPER2 * 2;
	}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	//HAL_ADC_Init(&hadc1);		//????
	//Калибровка АЦП
	//HAL_ADCEx_Calibration_Start(&hadc1, SingleDiff);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
	//HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	//HAL_ADCEx_Calibration_Start(&hadc1, uint32_t SingleDiff);
	//https://blog.radiotech.kz/perevod/adc-perevod-iz-knigi-mastering-stm32/

	//HAL_ADCEx_Calibration_Start(&hadc1);

	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_Start_DMA(&hadc1, &timme, flag);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc, 4);
	//HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);	//старт 2-го канала таймера №2
	HAL_TIM_Base_Start(&htim6);

	HAL_DAC_Start(&hdac1,DAC_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (HAL_GetTick() - T >= 500)
		{
			T = HAL_GetTick();
			HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			//HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
			LED_State = HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin);
			//flag = 1;

		}

		//REPER = (REPER +2) * 1;
		//REPER = 5;
		//flag = 1;	//УБРАТЬ!!!

		if (flag)
		{
			//HAL_Delay(250);	//УБРАТЬ!!!

			flag = 0;		//Восстановить!
			//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
			//LED_State = HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin);
			//_L = !_L;

			//puts("LED_State меняет своё значение");

			//for (uint8_t j = 0; j < 4; j++)
			//{
			//HAL_ADC_Stop_DMA(&hadc1); // <a> необязательно
			//snprintf(trans_str, 63, "ADC %d\n", adc[j]);
			//snprintf(trans_str, 63, "ADC %d \n", adc);
			//HAL_UART_Transmit(&huart2, (uint8_t*) trans_str,
			//strlen(trans_str), 1000);
			//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc, j);
			//HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc, 1);

			//printf("��� = %d.\n", adc[j]);
			//puts("АЦП работает правильно!");

			//printf("ADC = %d\n", adc[j]);
			//puts("ADC arbeit richtig");

			//adc[j] = 0;
			//REPER++;
			//HAL_Delay(100);	// >=100 мс. Спасает от сваливания в void HardFault_Handler(void)!!!
			//flag = 1;	//УБРАТЬ!!!
			//}
			//SYGNaL = adc[0];
			//Temp_Sens = adc[1];
			//Vbat = adc[2];
			//mcuVoltage = ADC_MAX * ADC_REFERENCE_VOLTAGE / adc[3];

			//HAL_Delay(5000);	//�?ли здесь. Величина влияет на "просечки", см. график на STM32CubeMonitor. Спасает от сваливания в void HardFault_Handler(void)!!!

			Temp_Sens = __LL_ADC_CALC_TEMPERATURE(VDDA_APPLI, adc[1],
					LL_ADC_RESOLUTION_12B);
			//Temp_DEG_C = (float) Temp_Sens / 10;

			/*Раскрытый макрос измерения температуры;*/
			/* (((( ((int32_t)((((((adc[1])) << ((((0x00000000UL))) >> (( 3UL) - 1UL))) >> (((0x00000000UL)) >> (( 3UL) - 1UL))) * ((3300U))) / (3000UL)) - (int32_t) *((uint16_t*) (0x1FFF75A8UL)))  \
 ) * (int32_t)((130L) - (( int32_t)   30L)) ) / (int32_t)((int32_t)*((uint16_t*) (0x1FFF75CAUL)) - (int32_t)*((uint16_t*) (0x1FFF75A8UL)))) + (( int32_t)   30L)); */

			//puts("TS_CAL1");
			//printf("TS_CAL1 = %hu\n", (int32_t)*((uint16_t*) (0x1FFF75A8UL)));
			//puts("TS_CAL2");
			//printf("TS_CAL2 = %hu\n", (int32_t)*((uint16_t*) (0x1FFF75CAUL)));
			//ПРОВЕР�?Л на калькуляторе
			//(130-30)/(1386-1045)*(940*3300/3000-1045)+30 = 26,77 ����.� �����!

			//Vrefint_mV   = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc[3], LL_ADC_RESOLUTION_12B);
			//Vref_V = (float) Vrefint_mV / 1000;
			Vref_V = (float) __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc[3],
					LL_ADC_RESOLUTION_12B) / 1000;

			//Vbat_mV   = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc[2], LL_ADC_RESOLUTION_12B) * 3;
			//Vbat_V = (float) Vbat_mV / 1000;
			Vbat_V = (float) __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc[2],
					LL_ADC_RESOLUTION_12B) * 3 / 1000;

			VDD_mV = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc[3],
					LL_ADC_RESOLUTION_12B);
			VDD_V = (float) VDD_mV / 1000;
			//VDD_V = (float) __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc[3], LL_ADC_RESOLUTION_12B) / 1000;

			//SYGNaL_single_mV    = (__LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc[0], LL_ADC_RESOLUTION_12B) - VDDA_APPLI/2) * 2;
			//SYGNaL_single_mV    = __LL_ADC_CALC_VREFANALOG_VOLTAGE(adc[0], LL_ADC_RESOLUTION_12B);
			//SYGNaL_dif_mV = SYGNaL_single_mV - Vrefint_mV;

			//SYGNaL_single_mV_hal = __HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, adc[0], ADC_RESOLUTION_12B);
			//SYGNaL_single_mV_hal = __HAL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, (float) (adc[0] - MSB_/2), ADC_RESOLUTION_12B) * 2;

			SYGNaL_dif_mV = (float) (adc[0] - MSB_ / 2) / (MSB_ / 2) * VDD_mV;
			//SYGNaL_dif_mV_ = (float) (adc[0] - 2048) / 2048 * VDD_mV;

			//puts("U_In");
			//printf("SYGNaL_single_mV = %d\n", SYGNaL_single_mV);

			//puts("U_In_hal");
			//printf("SYGNaL_single_mV_hal = %d\n", SYGNaL_single_mV_hal);

			//puts("U_Diff_In");
			//printf("SYGNaL_dif_mV = %2.1f\n", SYGNaL_dif_mV);

			//puts("Vrefint, mV");
			//printf("Vrefint_mV = %d\n", Vrefint_mV);

			//puts("Vref_V, V");
			//printf("Vref_V = %2.1f\n", Vref_V);

			//puts("Temp_Sens, Deg.C");
			//printf("Temp_Sens = %d\n", Temp_Sens);

			//puts("Temp_Sens, Deg.C");
			//printf("Temp_Sens = %2.1f\n", Temp_DEG_C);

			//puts("Vbat_mV, mV");
			//printf("Vbat_mV/3 = %d\n", Vbat_mV);

			//puts("Vbat_V, V");
			//printf("Vbat_V = %2.1f\n", Vbat_V);

			//puts("VDD_mV, mV");
			//printf("VDD_mV = %d\n", VDD_mV);

			//puts("VDD_V, V");
			//printf("VDD_V = %2.1f\n", VDD_V);

			//HAL_Delay(100);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adc[0]);
			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (adc[0] - MSB_ / 2));

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000000);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000280);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000500);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000780);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000999);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000780);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000500);

			//HAL_Delay(2);

			//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0x00000280);

			//HAL_Delay(2);

		}

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, adc[0]);

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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|PWM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin PWM_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//это стандартная запись для посимвольного вывода информации в интерфейс ITM в среде STM32CubeIDE
//(должно быть в случае применения SWO)
//https://www.youtube.com/watch?v=nE-YrKpWjso&list=PL9lkEHy8EJU8a_bqiJXwGTo-uM_cPa98P
int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return ch;
}

//или:

//STM32: Отладка через SWO в STM32CubeIDE с доработкой ST-LINK
//https://www.youtube.com/watch?v=ST_fUu6ACzE

//int _write(int file, char *ptr, int len)
 //{
 //int i = 0;
 //for(i = 0; i < len; i++)
    //ITM_SendChar((*ptr++));
 //return len;
 //}

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

