/**
  ******************************************************************************
  * @file           : main.c
  * @author			: Vaggelis Koutouloulis
  * @date			: June 2025
  * @brief          : Battery monitoring and protection system using INA219 sensors and STM32.
  ******************************************************************************
  *
  * This program measures voltage, current, and power across 3 battery cells
  * using INA219 sensors over I2C and protects the load from unsafe conditions.
  *
  * Developed by Vaggelis Koutouloulis as part of a personal engineering portfolio.
  *
  ******************************************************************************
  */
#include "main.h"
#include "ina219.h"
#include "stdio.h"

#define NUM_CELLS 3

// Thresholds
#define OV_THRESHOLD_V 4.2f     // Overvoltage
#define UV_THRESHOLD_V 3.0f     // Undervoltage
#define OC_THRESHOLD_mA 2000.0f // Overcurrent

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

// I2C addresses of the 3 INA219 modules (left-shifted for STM32 HAL)
const uint8_t ina_addresses[NUM_CELLS] = {
    INA219_ADDR_CELL1,
    INA219_ADDR_CELL2,
    INA219_ADDR_CELL3
};

// For printf over UART
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

int main(void)
{
	// Variables to track max/min values
	float max_voltage[NUM_CELLS];
	float min_voltage[NUM_CELLS];
	float max_current[NUM_CELLS];
	float min_current[NUM_CELLS];

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    //INA219_Init(&hi2c1);

    // Array of cells — one struct per INA219
    CellData cells[NUM_CELLS];

    // Ensure load is initially connected
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    for (int i = 0; i < NUM_CELLS; ++i) {
        max_voltage[i] = 0.0f;
        min_voltage[i] = 100.0f;      // High initial value for max
        max_current[i] = 0.0f;
        min_current[i] = 10000.0f;    // High initial value for min
    }

    // Main loop
    while (1)
    {

    	float voltage = 0.0f;
    	float current = 0.0f;

    	for (int i = 0; i < NUM_CELLS; ++i)
		{
			// Read voltage, current, power for this cell
			if (INA219_ReadAll(&hi2c1, ina_addresses[i], &cells[i]) == HAL_OK)
			{
				// Update min/max voltage
				if (cells[i].voltage_V > max_voltage[i]) max_voltage[i] = cells[i].voltage_V;
				if (cells[i].voltage_V < min_voltage[i]) min_voltage[i] = cells[i].voltage_V;

				// Update min/max current
				if (cells[i].current_mA > max_current[i]) max_current[i] = cells[i].current_mA;
				if (cells[i].current_mA < min_current[i]) min_current[i] = cells[i].current_mA;

				// Print current reading and min/max stats
				printf("Cell %d: V=%.3f V (Min: %.3f, Max: %.3f) | I=%.3f mA (Min: %.3f, Max: %.3f) | P=%.3f W\r\n",
					   i + 1,
					   cells[i].voltage_V, min_voltage[i], max_voltage[i],
					   cells[i].current_mA, min_current[i], max_current[i],
					   cells[i].power_W);

			// Basic protection status
			if (cells[i].voltage_V < UV_THRESHOLD_V)
				printf("  → Cell %d: Undervoltage!\r\n", i + 1);

			if (cells[i].voltage_V > OV_THRESHOLD_V)
				printf("  → Cell %d: Overvoltage!\r\n", i + 1);

			if (cells[i].current_mA > OC_THRESHOLD_mA)
				printf("  → Cell %d: Overcurrent!\r\n", i + 1);
			}
			else
			{
				printf("Cell %d: INA219 read failed.\r\n", i + 1);
			}

			CheckBatterySafety(cells[i].voltage_V, cells[i].current_mA);
		}

		printf("--------------------------------------------------\r\n");
		HAL_Delay(1000);
	}
}

// Function to control load based on measurements
/// @brief  Check battery parameters and enable/disable load accordingly.
/// @param  voltage    Last measured bus voltage (V)
/// @param  current_mA Last measured current (mA)
void CheckBatterySafety(float voltage, float current_mA)
{
	// 'static' retains its value between calls
    static uint8_t load_connected = 1;

    if (voltage > OV_THRESHOLD_V || voltage < UV_THRESHOLD_V || current_mA > OC_THRESHOLD_mA)
    {
        // Dangerous condition — disconnect load
        if (load_connected)
        {
        	// Drive PB0 LOW to open the MOSFET/relay
            HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_RESET);
            load_connected = 0;
            printf("Protection triggered: Load disconnected\r\n");
        }
    }
    else
    {
        // Conditions safe — reconnect if needed
        if (!load_connected) {
            HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_SET);
            load_connected = 1;
            printf("Conditions safe: Load reconnected\r\n");
        }
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LOAD_EN_Pin */
  GPIO_InitStruct.Pin = LOAD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LOAD_EN_GPIO_Port, &GPIO_InitStruct);

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
