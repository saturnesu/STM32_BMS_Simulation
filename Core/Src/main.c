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
#include "ina219_multi.h"
#include "tmp102_multi.h"
#include "stdio.h"
#include "ssd1306.h"
#include "max17043.h"
#include "tca9548a.h"

#define NUM_CELLS 3

// Thresholds
#define OV_THRESHOLD_V 4.2f     // Overvoltage
#define UV_THRESHOLD_V 3.0f     // Undervoltage
#define OC_THRESHOLD_mA 2000.0f // Overcurrent
#define OT_THRESHOLD_C 60.0f	// Overtemperature

typedef enum {
    BMS_STATE_IDLE = 0,
    BMS_STATE_CHARGING,
    BMS_STATE_DISCHARGING,
    BMS_STATE_FAULT
} BMS_State_t;

typedef struct {
    CellData data;
    uint8_t fault;
    float temperature_C;
    float soc_percent;
} CellStatus;

static BMS_State_t currentState = BMS_STATE_IDLE;

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

// I2C addresses of the 3 INA219 (left-shifted for STM32 HAL)
const uint8_t ina_addresses[NUM_CELLS] = {
    INA219_ADDR_CELL1,
    INA219_ADDR_CELL2,
    INA219_ADDR_CELL3
};

// I2C addresses of the 3 TMP102 (left-shifted for STM32 HAL)
const uint8_t tmp102_addresses[NUM_CELLS] = {
    TMP102_ADDR_CELL1,
    TMP102_ADDR_CELL2,
    TMP102_ADDR_CELL3
};

// For printf over UART
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void SystemClock_Config(void);
void CheckBatterySafety(float voltage, float current_mA, float temperature_C);
void DisplayToOLED(CellStatus cells[], int num_cells);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    SSD1306_Init(&hi2c1);
    SSD1306_Clear();
    SSD1306_UpdateScreen();

    for (int i = 0; i < NUM_CELLS; ++i) {
        MAX17043_SelectChannel(&hi2c1, i);
        MAX17043_QuickStart(&hi2c1);
    }

    //INA219_Init(&hi2c1);

    // Array of cells — one struct per INA219
    CellStatus cells[NUM_CELLS];

    // Variables to track max/min values
    float max_voltage[NUM_CELLS] = {0};
    float min_voltage[NUM_CELLS] = {100.0f};	// High initial value for max
    float max_current[NUM_CELLS] = {0};
    float min_current[NUM_CELLS] = {10000.0f};	// High initial value for min

    // Ensure load is initially connected
    HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_SET);

    // Main loop
    while (1) {
        for (int i = 0; i < NUM_CELLS; ++i) {
            cells[i].fault = 0;
            // Read voltage, current, power for this cell
            if (INA219_ReadAll(&hi2c1, ina_addresses[i], &cells[i].data) == HAL_OK) {
                float V = cells[i].data.voltage_V;
                float I = cells[i].data.current_mA;

                // Read Temperature
                float T = 0.0f;
				if (TMP102_ReadTemperature(&hi2c1, tmp102_addresses[i], &T) == HAL_OK) {
					cells[i].temperature_C = T;
				}
				else  {
					printf("Cell %d: TMP102 read failed\r\n", i + 1);
					cells[i].fault = 1;
					continue;
				}

                // Update min/max voltage
                if (V > max_voltage[i]) max_voltage[i] = V;
                if (V < min_voltage[i]) min_voltage[i] = V;
                if (I > max_current[i]) max_current[i] = I;
                if (I < min_current[i]) min_current[i] = I;

                // Print current reading and min/max stats
                printf("Cell %d: V=%.3f V | I=%.3f mA | T=%.2f °C | P=%.3f W\r\n", i + 1, V, I, T, cells[i].data.power_W);

                //Fault detection
                if (V > OV_THRESHOLD_V || V < UV_THRESHOLD_V || I > OC_THRESHOLD_mA || T > OT_THRESHOLD_C) {
                    cells[i].fault = 1;
                }
                // Control load
                CheckBatterySafety(V, I, T);
            } else {
                printf("Cell %d: INA219 read failed.\r\n", i + 1);
                cells[i].fault = 1;
            }

            // Read SoC from MAX17043
    		float soc = 0.0f;
    		MAX17043_SelectChannel(&hi2c1, i);
    		if (MAX17043_ReadSOC(&hi2c1, &soc) == HAL_OK) {
    			cells[i].soc_percent = soc;
    		} else {
    			printf("Cell %d: MAX17043 read failed\r\n", i + 1);
    			cells[i].fault = 1;
    		}
        }

        DisplayToOLED(cells, NUM_CELLS);

        // ----------- BMS STATE LOGIC --------------
        switch (currentState) {
            case BMS_STATE_IDLE:
                for (int i = 0; i < NUM_CELLS; ++i) {
                    float I = cells[i].data.current_mA;
                    if (I < -50.0f) {
                        currentState = BMS_STATE_CHARGING;
                        printf("→ State: CHARGING\r\n");
                        break;
                    } else if (I > 50.0f) {
                        currentState = BMS_STATE_DISCHARGING;
                        printf("→ State: DISCHARGING\r\n");
                        break;
                    }
                }
                break;

            case BMS_STATE_CHARGING:
            case BMS_STATE_DISCHARGING:
                for (int i = 0; i < NUM_CELLS; ++i) {
                    if (cells[i].fault) {
                        currentState = BMS_STATE_FAULT;
                        printf("→ FAULT detected (Cell %d)\r\n", i + 1);
                        break;
                    }
                }
                break;

            case BMS_STATE_FAULT:
                printf("→ System in FAULT: Disconnecting load.\r\n");
                HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_RESET);
                break;
        }

        printf("--------------------------------------------------\r\n");
        HAL_Delay(1000);
    }
}

// Function to control load based on measurements
/// @brief  Check battery parameters and enable/disable load accordingly.
/// @param  voltage    Last measured bus voltage (V)
/// @param  current_mA Last measured current (mA)
void CheckBatterySafety(float voltage, float current_mA, float temperature_C)
{
		// 'static' retains its value between calls
		static uint8_t load_connected = 1;

		if (voltage > OV_THRESHOLD_V || voltage < UV_THRESHOLD_V || current_mA > OC_THRESHOLD_mA || temperature_C > OT_THRESHOLD_C)
		{
			// Dangerous condition — disconnect load
			if (load_connected)
			{
				// Drive PB0 LOW to open the MOSFET
				HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_RESET);
				load_connected = 0;
				printf("Protection triggered: Load disconnected\r\n");
			}
		}
		else
		{
			// Conditions safe — reconnect if needed
			if (!load_connected)
		{
			HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_SET);
			load_connected = 1;
			printf("Conditions safe: Load reconnected\r\n");
		}
	}
}

// Charging Bar function (0-100%) to OLED, width 40px
static void DrawChargeBar(int x, int y, float percent) {
    int barWidth = 40;
    int fillWidth = (int)((percent / 100.0f) * barWidth);
    SSD1306_DrawRect(x, y, barWidth, 7, 1);
    if (fillWidth > 0) {
        SSD1306_FillRect(x + 1, y + 1, fillWidth - 2, 5, 1);
    }
}

void DisplayToOLED(CellStatus cells[], int num_cells) {
    SSD1306_Clear();

    char line[16];

    for (int i = 0; i < num_cells; ++i) {
        snprintf(line, sizeof(line), "C%d", i + 1);
        SSD1306_GotoXY(0 + i * 42, 0);
        SSD1306_Puts(line, &Font_6x8, 1);
    }

    for (int i = 0; i < num_cells; ++i) {
        snprintf(line, sizeof(line), "%.2fV", cells[i].data.voltage_V);
        SSD1306_GotoXY(0 + i * 42, 10);
        SSD1306_Puts(line, &Font_6x8, 1);
    }

    for (int i = 0; i < num_cells; ++i) {
        snprintf(line, sizeof(line), "%.0fmA", cells[i].data.current_mA);
        SSD1306_GotoXY(0 + i * 42, 20);
        SSD1306_Puts(line, &Font_6x8, 1);
    }

    for (int i = 0; i < num_cells; ++i) {
        snprintf(line, sizeof(line), "%.1fC", cells[i].temperature_C);
        SSD1306_GotoXY(0 + i * 42, 30);
        SSD1306_Puts(line, &Font_6x8, 1);
    }

    for (int i = 0; i < num_cells; ++i) {
        snprintf(line, sizeof(line), "%s", cells[i].fault ? "FAULT" : "OK");
        SSD1306_GotoXY(0 + i * 42, 40);
        SSD1306_Puts(line, &Font_6x8, 1);
    }

    // Display SOC % with number and charging bar
    for (int i = 0; i < num_cells; ++i) {
        snprintf(line, sizeof(line), "%.0f%%", cells[i].soc_percent);
        SSD1306_GotoXY(0 + i * 42, 50);
        SSD1306_Puts(line, &Font_6x8, 1);

        DrawChargeBar(0 + i * 42, 57, cells[i].soc_percent);
    }

    const char *state_str = "";
    switch (currentState) {
        case BMS_STATE_IDLE:         state_str = "IDLE"; break;
        case BMS_STATE_CHARGING:     state_str = "CHARGING"; break;
        case BMS_STATE_DISCHARGING:  state_str = "DISCHARGING"; break;
        case BMS_STATE_FAULT:        state_str = "FAULT"; break;
    }
    snprintf(line, sizeof(line), "State: %s", state_str);
    SSD1306_GotoXY(0, 48);
    SSD1306_Puts(line, &Font_6x8, 1);

    SSD1306_UpdateScreen();
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

  HAL_GPIO_WritePin(LOAD_EN_GPIO_Port, LOAD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Cell_2_Status_Pin Cell_3_Status_Pin Cell_1_Status_Pin */
  GPIO_InitStruct.Pin =Cell_1_Status_Pin|Cell_2_Status_Pin|Cell_3_Status_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
