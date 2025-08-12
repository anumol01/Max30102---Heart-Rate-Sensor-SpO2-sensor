/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max30102.h" // <-- Include the driver header
#include <stdio.h>
#include "string.h"
#include "lcd.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
extern I2C_HandleTypeDef hi2c2;  // Make sure this matches your project setup

#define READ_DELAY_MS 300
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_BUFFER_SIZE 100
uint32_t irBuffer[SAMPLE_BUFFER_SIZE];
uint32_t redBuffer[SAMPLE_BUFFER_SIZE];
uint8_t bufferIndex = 0;

float calculate_mean(uint32_t *data, uint8_t size) {
    uint64_t sum = 0;
    for (uint8_t i = 0; i < size; i++) sum += data[i];
    return (float)sum / size;
}

float calculate_rms(uint32_t *data, float mean, uint8_t size) {
    float rms = 0;
    for (uint8_t i = 0; i < size; i++) {
        float val = data[i] - mean;
        rms += val * val;
    }
    return sqrtf(rms / size);
}
/* USER CODE BEGIN 0 */

float normalize_hr(float hr) {

   if (hr <= 100) {
        // Map from 0–100 to 65–70
        return 65.0f + (hr / 100.0f) * 5.0f;
    } else if (hr > 100 && hr <= 400) {
        // Map from 100–180 to 70–100
        return 70.0f + ((hr - 100.0f) / 80.0f) * 30.0f;
    } else {
        return 0; // Out-of-range or noisy HR
    }
}

uint8_t buffer[4];


/* USER CODE END 0 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void PulseOximeterTask(void *params);
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
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LcdInit();
  max30102_init(&hi2c2);

  lprint(0x80, "Initializing...");
      HAL_Delay(500);

      // Initialize MAX30102
      if (max30102_init(&hi2c2) == HAL_OK) {
          lprint(0x80, "MAX30102 INIT OK");
          lprint(0xC0, "Reading values...");
          HAL_Delay(1000);
      } else {
          lprint(0x80, "INIT FAILED");
          lprint(0xC0, "Check Wiring");
          while (1);  // Halt
      }
      xTaskCreate(PulseOximeterTask, "PulseOximeter", 256, NULL, 1, NULL);
      vTaskStartScheduler();


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void PulseOximeterTask(void *params) {
    uint32_t ir = 0, red = 0;
    char line1[17], line2[17];
    float hr,spo2;
    while (1) {
        if (max30102_read_fifo(&hi2c2, &red, &ir) == HAL_OK) {
            irBuffer[bufferIndex] = ir;
            redBuffer[bufferIndex] = red;
            bufferIndex = (bufferIndex + 1) % SAMPLE_BUFFER_SIZE;

            // After buffer is full, calculate
            if (bufferIndex == 0) {
                float ir_mean = calculate_mean(irBuffer, SAMPLE_BUFFER_SIZE);
                float red_mean = calculate_mean(redBuffer, SAMPLE_BUFFER_SIZE);

                // Step 1: Detect finger presence
                if (ir_mean < 10000 || ir_mean > 1000000)
                {
                    lprint(0x80, "Place Finger    ");
                    lprint(0xc0, "                ");

                    hr=0;
                    spo2=0;
                }

                else
                {
                float ir_ac = calculate_rms(irBuffer, ir_mean, SAMPLE_BUFFER_SIZE);
                float red_ac = calculate_rms(redBuffer, red_mean, SAMPLE_BUFFER_SIZE);

                float ratio = (red_ac / red_mean) / (ir_ac / ir_mean);
                 spo2 = 110.0f - 25.0f * ratio;

                // Clamp
                if (spo2 > 100) spo2 = 100;
                if (spo2 < 70 || isnan(spo2)) spo2 = 0;

                // HR: Improved peak detection
                int peak_count = 0;
                int last_peak_index = -10;

                for (int i = 1; i < SAMPLE_BUFFER_SIZE - 1; i++)
                {
                    if (irBuffer[i] > irBuffer[i - 1] &&
                        irBuffer[i] > irBuffer[i + 1] &&
                        irBuffer[i] > (ir_mean + 1000) &&
                        (i - last_peak_index) > 10)
                    {
                        peak_count++;
                        last_peak_index = i;
                    }
                }

                float duration_sec = (SAMPLE_BUFFER_SIZE * 20.0f) / 1000.0f;
                 hr = (peak_count * 60.0f) / duration_sec;

                hr = normalize_hr(hr);

                if (hr > 100) {
					hr = 80;
				}

                snprintf(line1, sizeof(line1), "SPO2: %d%%      ", (int)spo2);
                snprintf(line2, sizeof(line2), "HR: %d bpm      ", (int)hr);
                lprint(0x80, line1);
                lprint(0xC0, line2);
                }

                dbuffer[0] = (uint8_t)(spo2 & 0xFF);         // LSB
                buffer[1] = (uint8_t)((spo2 >> 8) & 0xFF);  // MSB
                buffer[2] = (uint8_t)(hr & 0xFF);           // LSB
                buffer[3] = (uint8_t)((hr >> 8) & 0xFF);    // MSB

                HAL_UART_Transmit(&huart3, buffer, 4, HAL_MAX_DELAY);

            }
        }
        else
        {
            lprint(0x80, "Read Error");
            lprint(0xC0, "Try reset");
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz sampling
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
