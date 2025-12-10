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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define _USE_SHT31
#define _USE_AHT20
#define _USE_BME280
#ifdef _USE_SHT31
	#include <../../sht31/SHT31.h>
#endif
#ifdef _USE_HTU21DF
	#include <../../htu21df/HTU21DF.h>
#endif
#ifdef _USE_AHT20
	#include <../../aht20/AHT20.h>
#endif
#ifdef _USE_BMP280
	#include <../../bmp280/bmp280.h>
#endif
#ifdef _USE_BME280
	#include "../../bme280/BME280.h"
	using namespace BME280;
#endif
#include <math.h>
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float t,h;
double temperature,humidity;
double PressureOffset=0.10;
double p,pressure;
uint8_t charBuffer[255];
#ifdef _USE_HTU21DF
	Adafruit_HTU21DF htu21df(&hi2c1);
#endif
#ifdef _USE_AHT20
	float AHT20_temp,AHT20_humidity;
	AHT20 AHT20_sensor(&hi2c1);
#endif
#ifdef _USE_BMP280
	using namespace BMP280;
	bmp280i2c sensor(&hi2c1,bmp280_i2c_addr_2);
#endif

#ifdef _USE_BME280
	double BME280_temperature,BME280_humidity,BME280_pressure;
	bme280i2c BME280_sensor(&hi2c1,bme280_i2c_addr_2);
#endif

#ifdef _USE_SHT31
	float SHT31_temp,SHT31_humidity;
	SHT31_param_t sht31_param = {
			.hi2c=&hi2c1,
			.i2c_addr =sht31_i2c_addr1,
			.periodic_mode=true,
			.heater_enable = false,
			.sample_rate=_05mps_high_Res
	};
	SHT31 SHT31_sensor(sht31_param);
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef _USE_HTU21DF

    t=htu21df.readTemperature();

            if (! isnan(t)) {  // check if 'is not a number'
              temperature = t;
              //printf("");
              sprintf((char*)charBuffer,"Temp:  %f\t\t",temperature);
              //printf("\t\t");
              HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
            } else {
              printf("Failed to read temperature");
            }
     h=htu21df.readHumidity();
            if (! isnan(h)) {  // check if 'is not a number'
              humidity = h;
             // printf("");
              sprintf((char*)charBuffer,"RH = %f\r\n",humidity);
              HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
            } else {
              //printf("Failed to read humidity");
            }
#endif

#ifdef _USE_AHT20

                   AHT20_sensor.main();
                   if(AHT20_sensor.newData()){
                	   AHT20_sensor.getTempHumidity(&AHT20_temp,&AHT20_humidity);
               		sprintf((char*)charBuffer,"AHT20: \t");
               		HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
                	   if (! isnan(AHT20_humidity)) {  // check if 'is not a number'
                	                           // printf("");
                	                           sprintf((char*)charBuffer,"RH = %f\t\t",AHT20_humidity);
                	                           HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
                	                       } else {
                	                           //printf("Failed to read humidity");
                	                       }

                	   if (! isnan(AHT20_temp)) {  // check if 'is not a number'
                		   AHT20_temp = AHT20_temp*9/5+32;

                		   /* Write temperature to Debug UART */
                		   sprintf((char*)charBuffer,"Temp:  %f\r\n",AHT20_temp);
                		   HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
                	   } else {
                		   printf("Failed to read temperature");
                	   }
                   }


#endif

#ifdef _USE_Si7021

                   sensor.StartMeasurement();
                   HAL_Delay(100);
                   sensor.ReadTempHumidity(&t,&h);
                   if (! isnan(h)) {  // check if 'is not a number'
                        // printf("");
                        sprintf((char*)charBuffer,"RH = %f\r\n",h);
                        HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
                    } else {
                        //printf("Failed to read humidity");
                    }

                   if (! isnan(t)) {  // check if 'is not a number'
                	   t = t*9/5+32;
                                 /* Write temperature to Debug UART */
                                 sprintf((char*)charBuffer,"Temp:  %f\t\t",t);
                                 HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
                               } else {
                                 printf("Failed to read temperature");
                           }

#endif

#ifdef _USE_BMP280
                   sensor.main();
                   if(sensor.newData()){

                		sensor.GetTemperature(&t);
                		sensor.GetPressure(&p);

					   if (! isnan(p)) {  // check if 'is not a number'
							// printf("");
						   p= p*0.00029529983071445 + 0.06;
							sprintf((char*)charBuffer,"Pressure = %f\r\n",p);
							HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
						} else {
							//printf("Failed to read humidity");
						}

					   if (! isnan(t)) {  // check if 'is not a number'
						   t = t*9/5+32;
									 /* Write temperature to Debug UART */
									 sprintf((char*)charBuffer,"Temp:  %f\t\t",t);
									 HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
								   } else {
									 printf("Failed to read temperature");
							   }
                   }

#endif

#ifdef _USE_BME280
                   BME280_sensor.main();
                   if(BME280_sensor.newData()){

                	   BME280_sensor.GetTemperature(&BME280_temperature);
                	   BME280_sensor.GetPressure(&BME280_pressure);
                	   BME280_sensor.GetHumidity(&BME280_humidity);

                		sprintf((char*)charBuffer,"BME280: \t");
                		HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);

					   if (! isnan(BME280_pressure)) {  // check if 'is not a number'
							// printf("");
						   BME280_pressure= BME280_pressure*0.00029529983071445 + 0.12;
							sprintf((char*)charBuffer,"Pressure = %f\r\n",BME280_pressure);
							HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
						} else {
							//printf("Failed to read humidity");
						}

					   if (! isnan(BME280_temperature)) {  // check if 'is not a number'
						   BME280_temperature = BME280_temperature*9/5+32;
									 /* Write temperature to Debug UART */
									 sprintf((char*)charBuffer,"Temp:  %f\t\t",BME280_temperature);
									 HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
								   } else {
									 printf("Failed to read temperature");
							   }
					   if (! isnan(BME280_humidity)) {  // check if 'is not a number'
					                           // printf("");
					                           sprintf((char*)charBuffer,"RH = %f\r\n",BME280_humidity);
					                           HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
					                       } else {
					                           //printf("Failed to read humidity");
					                       }
                   }

#endif

#ifdef _USE_SHT31
                   SHT31_sensor.main();
                   if(SHT31_sensor.newData()){

                		SHT31_sensor.GetTemperature(&SHT31_temp);
                		SHT31_sensor.GetHumidity(&SHT31_humidity);

                		sprintf((char*)charBuffer,"SHT31 \t");
                		HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);

					   if (! isnan(SHT31_temp)) {  // check if 'is not a number'
						   SHT31_temp = SHT31_temp*9/5+32;
									 /* Write temperature to Debug UART */
									 sprintf((char*)charBuffer,"Temp:  %f\t\t",SHT31_temp);
									 HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
								   } else {
									 printf("Failed to read temperature");
							   }
					   if (! isnan(SHT31_humidity)) {  // check if 'is not a number'
					                           sprintf((char*)charBuffer,"RH = %f\r\n",SHT31_humidity);
					                           HAL_UART_Transmit(&huart2,charBuffer,strlen((char*)charBuffer),HAL_MAX_DELAY);
					                       } else {
					                           //printf("Failed to read humidity");
					                       }
                   }

#endif


    HAL_Delay(50);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00000E14;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
