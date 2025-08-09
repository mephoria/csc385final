#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c2;  // Fixed: Match the peripheral name

#define LSM6DSL_ADDR (0x6B << 1) // 8-bit HAL address
#define WHO_AM_I     0x0F
#define CTRL1_XL     0x10
#define CTRL2_G      0x11
#define OUTX_L_G     0x22
#define OUTX_L_XL    0x28

void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);

static HAL_StatusTypeDef imu_write(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(&hi2c2, LSM6DSL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef imu_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_USART1_UART_Init();
    MX_I2C2_Init();

    char msg[200];
    HAL_StatusTypeDef status;

    HAL_UART_Transmit(&huart1, (uint8_t*)"Starting I2C diagnostics...\r\n", 30, HAL_MAX_DELAY);

    // Try both possible I2C addresses
    uint8_t addresses[] = {0x6A << 1, 0x6B << 1}; // 0xD4, 0xD6
    uint8_t found_addr = 0;
    
    for (int i = 0; i < 2; i++) {
        status = HAL_I2C_IsDeviceReady(&hi2c2, addresses[i], 3, 100);
        sprintf(msg, "Address 0x%02X: %s\r\n", addresses[i], 
                status == HAL_OK ? "FOUND" : "Not responding");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        
        if (status == HAL_OK) {
            found_addr = addresses[i];
            break;
        }
    }

    if (found_addr == 0) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR: No device found! Check connections.\r\n", 45, HAL_MAX_DELAY);
        // Continue anyway to help debug
        found_addr = LSM6DSL_ADDR;
    }

    // Update global address if different
    #undef LSM6DSL_ADDR
    #define LSM6DSL_ADDR found_addr

    // Check WHO_AM_I with the found address
    uint8_t who = 0;
    status = HAL_I2C_Mem_Read(&hi2c2, found_addr, WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &who, 1, 100);
    sprintf(msg, "WHO_AM_I: 0x%02X (Expected: 0x6A) - Status: %d\r\n", who, status);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    
    if (who == 0x6A) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"✓ LSM6DSL detected successfully!\r\n", 35, HAL_MAX_DELAY);
    } else if (who == 0x00 || who == 0xFF) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"⚠ Communication issue, but sensor responding\r\n", 47, HAL_MAX_DELAY);
    } else {
        sprintf(msg, "⚠ Different sensor detected (ID: 0x%02X)\r\n", who);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }

    if (status != HAL_OK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"CRITICAL: Cannot communicate with sensor!\r\n", 44, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart1, (uint8_t*)"Check: 1) Wiring 2) Power 3) Pullup resistors\r\n", 49, HAL_MAX_DELAY);
    }

    // Try simple configuration first
    HAL_Delay(10);
    status = HAL_I2C_Mem_Write(&hi2c2, found_addr, CTRL1_XL, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x60}, 1, 100);
    sprintf(msg, "Accel config status: %d\r\n", status);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    HAL_Delay(10);
    status = HAL_I2C_Mem_Write(&hi2c2, found_addr, CTRL2_G, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){0x60}, 1, 100);
    sprintf(msg, "Gyro config status: %d\r\n", status);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Verify configuration by reading back
    uint8_t ctrl1, ctrl2;
    HAL_I2C_Mem_Read(&hi2c2, found_addr, CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &ctrl1, 1, 100);
    HAL_I2C_Mem_Read(&hi2c2, found_addr, CTRL2_G, I2C_MEMADD_SIZE_8BIT, &ctrl2, 1, 100);
    sprintf(msg, "Config readback - CTRL1_XL: 0x%02X, CTRL2_G: 0x%02X\r\n", ctrl1, ctrl2);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    HAL_UART_Transmit(&huart1, (uint8_t*)"Starting measurements...\r\n", 26, HAL_MAX_DELAY);
    HAL_Delay(100);

    while (1) {
        uint8_t g[6], a[6];
        
        // Read gyroscope data with shorter timeout
        status = HAL_I2C_Mem_Read(&hi2c2, found_addr, OUTX_L_G, I2C_MEMADD_SIZE_8BIT, g, 6, 50);
        if (status != HAL_OK) {
            sprintf(msg, "Error reading gyro: %d (HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3)\r\n", status);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_Delay(500); // Longer delay on error
            continue;
        }

        // Read accelerometer data with shorter timeout
        status = HAL_I2C_Mem_Read(&hi2c2, found_addr, OUTX_L_XL, I2C_MEMADD_SIZE_8BIT, a, 6, 50);
        if (status != HAL_OK) {
            sprintf(msg, "Error reading accel: %d\r\n", status);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            HAL_Delay(500); // Longer delay on error
            continue;
        }

        // Convert raw data to signed 16-bit values
        int16_t gx = (int16_t)(g[1] << 8 | g[0]);
        int16_t gy = (int16_t)(g[3] << 8 | g[2]);
        int16_t gz = (int16_t)(g[5] << 8 | g[4]);

        int16_t ax = (int16_t)(a[1] << 8 | a[0]);
        int16_t ay = (int16_t)(a[3] << 8 | a[2]);
        int16_t az = (int16_t)(a[5] << 8 | a[4]);

        sprintf(msg, "G: %6d %6d %6d  A: %6d %6d %6d\r\n", gx, gy, gz, ax, ay, az);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(200); // Slower rate for debugging
    }
}

/* === Peripheral Init Functions === */
static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void MX_I2C2_Init(void) {
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11; // SCL, SDA
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x10909CEC; // 100 kHz (slower, more reliable)
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        // Initialization Error - could add error handling here
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

/* 80 MHz system clock config */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}