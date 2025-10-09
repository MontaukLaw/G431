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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "user_comm.h" // Include user-defined communication header
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_I2C3_Init();
    MX_ADC2_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */
    // HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET); // Turn on blue LED
    HAL_Delay(300);
    HAL_GPIO_WritePin(GPIOA, POWER_CTRL_Pin, GPIO_PIN_SET);
    // init_frame_tail();

    fill_tx_data();

    start_uart_rx();

    // 切换adc通道
    set_adc_ch(0);

    // 配置 SCL=PA8, SDA=PC11
    // IIC_Init();

    // ICM42688DelayMs(2);
    // bsp_Icm42688Init();
    icm42688_init();
    // init_42688();

    // icm42688_reset_quat_identity();
    // float g_q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // 初始四元数
    uint32_t last = HAL_GetTick();

    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    // 打开adc2_dma
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)bat_val_dma_buf, ADC2_DMA_BUF_LEN);

    // 打开tim7
    HAL_TIM_Base_Start_IT(&htim7);
    uint8_t tx_buf[100];

    start_adc_collecting();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {

        bat_task();

        led_task();

        key_task();

        gsensor_task();

        main_task_adc_first();

        bl_task();

        u2_task();

        charge_state_task();

        imu_rest_cmd_task();

#if 0

        // if (tim7_counter > 1000)
        // {
        //     tim7_counter = 0;
        //     sprintf(tx_buf, "running\r\n");
        //     HAL_UART_Transmit_DMA(&huart1, tx_buf, strlen((char *)tx_buf));
        // }
        // main_task_adc_first();
        HAL_Delay(10);

        icm42688_read_accel(&acc);
        icm42688_read_gyro(&gyro);

        uint32_t now = HAL_GetTick();
        float dt = (now - last) * 0.001f; // s
        last = now;

        icm42688_pipeline_update(&acc, &gyro, dt, g_q);

        char tx_buf[96];
        int n = snprintf(tx_buf, sizeof(tx_buf),
                         "%.4f, %.4f, %.4f, %.4f\r\n",
                         g_q[0], g_q[1], g_q[2], g_q[3]);
        if (n > 0)
        {
            HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, (uint16_t)n, 50);
        }
#endif
        // uint8_t tx_buf[100];
        // sprintf(tx_buf, "%.4f, %.4f, %.4f, %.4f\r\n", g_q[0], g_q[1], g_q[2], g_q[3]);
        // HAL_UART_Transmit_DMA(&huart1, tx_buf, strlen((char *)tx_buf)); // 发送四元数数据

        // uint8_t tx_end_buf[2] = {0x0d, 0x0a}; // 发送结束符
        // HAL_UART_Transmit_DMA(&huart1, tx_end_buf, sizeof(tx_end_buf)); // 发送结束符
        // icm42688_raw_to_units(&acc, &gyro, &acc_g, &gyro_dps);

        // // acc_g.*, gyro_dps.* 可直接用
        // HAL_Delay(10); // 100Hz 周期

        // uint8_t tx_buf[] = {0x01, 0x02, 0x0d, 0x0a};
        // HAL_UART_Transmit(&huart2, tx_buf, sizeof(tx_buf), 0xffff);
        // main_task_adc_first();

        // uart_send();
        // HAL_Delay(100); // Delay to simulate processing time
        // com_task();

        //
        // HAL_Delay(100);

        // main_task_adc();
        // points_data[0]++;
        // uart_send();
        // HAL_Delay(100); // Delay to simulate processing time
        // main_task_adc();
        // main_task();
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
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
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

#ifdef USE_FULL_ASSERT
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
