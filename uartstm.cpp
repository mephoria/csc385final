// STM32 Command Interface & Telemetry System
#include "stm32l4xx_hal.h"
#include <string.h>
#include <stdio.h>

// Command buffer for receiving commands
#define CMD_BUFFER_SIZE 128
char cmd_buffer[CMD_BUFFER_SIZE];
uint8_t cmd_index = 0;

// System state structure for telemetry
typedef struct {
    uint32_t uptime_ms;
    uint8_t imu_status;
    uint16_t imu_error_count;
    float battery_voltage;
    uint8_t system_mode;
    int16_t temperature;
    uint32_t command_count;
    uint32_t telemetry_count;
} SystemTelemetry_t;

SystemTelemetry_t telemetry = {0};

// Command types
typedef enum {
    CMD_UNKNOWN = 0,
    CMD_GET_STATUS,
    CMD_SET_MODE,
    CMD_CALIBRATE_IMU,
    CMD_SET_REPORTING_RATE,
    CMD_RESET_SYSTEM,
    CMD_GET_IMU_DATA,
    CMD_SET_LED,
    CMD_GET_TELEMETRY
} CommandType_t;

// System modes
typedef enum {
    MODE_IDLE = 0,
    MODE_IMU_STREAMING,
    MODE_SERVO_CONTROL,
    MODE_CALIBRATION,
    MODE_DIAGNOSTIC
} SystemMode_t;

// Function prototypes
void process_uart_command(char* command);
void send_response(const char* response);
void send_telemetry(void);
void send_imu_data(void);
CommandType_t parse_command(char* cmd);

// UART receive interrupt handler
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        uint8_t received_char;
        HAL_UART_Receive_IT(&huart1, &received_char, 1);
        
        if (received_char == '\r' || received_char == '\n') {
            if (cmd_index > 0) {
                cmd_buffer[cmd_index] = '\0';
                process_uart_command(cmd_buffer);
                cmd_index = 0;
            }
        } else if (cmd_index < CMD_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = received_char;
        }
    }
}

// Command processing function
void process_uart_command(char* command) {
    telemetry.command_count++;
    CommandType_t cmd_type = parse_command(command);
    
    switch (cmd_type) {
        case CMD_GET_STATUS:
            send_response("STATUS:OK,MODE:%d,UPTIME:%lu", 
                         telemetry.system_mode, telemetry.uptime_ms);
            break;
            
        case CMD_SET_MODE: {
            int mode = atoi(strchr(command, ':') + 1);
            if (mode >= 0 && mode <= MODE_DIAGNOSTIC) {
                telemetry.system_mode = mode;
                send_response("ACK:MODE_SET");
            } else {
                send_response("ERROR:INVALID_MODE");
            }
            break;
        }
        
        case CMD_CALIBRATE_IMU:
            // Perform IMU calibration
            send_response("ACK:CALIBRATION_STARTED");
            // ... calibration code ...
            send_response("STATUS:CALIBRATION_COMPLETE");
            break;
            
        case CMD_SET_REPORTING_RATE: {
            int rate = atoi(strchr(command, ':') + 1);
            // Set reporting rate (1-1000 Hz)
            send_response("ACK:RATE_SET:%d", rate);
            break;
        }
        
        case CMD_GET_IMU_DATA:
            send_imu_data();
            break;
            
        case CMD_GET_TELEMETRY:
            send_telemetry();
            break;
            
        case CMD_SET_LED: {
            char* state = strchr(command, ':') + 1;
            if (strcmp(state, "ON") == 0) {
                // Turn LED on
                send_response("ACK:LED_ON");
            } else if (strcmp(state, "OFF") == 0) {
                // Turn LED off
                send_response("ACK:LED_OFF");
            } else {
                send_response("ERROR:INVALID_LED_STATE");
            }
            break;
        }
        
        case CMD_RESET_SYSTEM:
            send_response("ACK:RESETTING");
            HAL_Delay(100);
            NVIC_SystemReset();
            break;
            
        default:
            send_response("ERROR:UNKNOWN_COMMAND");
            break;
    }
}

// Parse incoming command
CommandType_t parse_command(char* cmd) {
    if (strncmp(cmd, "GET_STATUS", 10) == 0) return CMD_GET_STATUS;
    if (strncmp(cmd, "SET_MODE:", 9) == 0) return CMD_SET_MODE;
    if (strncmp(cmd, "CALIBRATE", 9) == 0) return CMD_CALIBRATE_IMU;
    if (strncmp(cmd, "SET_RATE:", 9) == 0) return CMD_SET_REPORTING_RATE;
    if (strncmp(cmd, "GET_IMU", 7) == 0) return CMD_GET_IMU_DATA;
    if (strncmp(cmd, "GET_TELEMETRY", 13) == 0) return CMD_GET_TELEMETRY;
    if (strncmp(cmd, "SET_LED:", 8) == 0) return CMD_SET_LED;
    if (strncmp(cmd, "RESET", 5) == 0) return CMD_RESET_SYSTEM;
    return CMD_UNKNOWN;
}

// Send response back to host
void send_response(const char* format, ...) {
    char response[256];
    va_list args;
    va_start(args, format);
    vsnprintf(response, sizeof(response), format, args);
    va_end(args);
    
    strcat(response, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
}

// Send comprehensive telemetry
void send_telemetry(void) {
    telemetry.telemetry_count++;
    telemetry.uptime_ms = HAL_GetTick();
    
    char telemetry_msg[512];
    snprintf(telemetry_msg, sizeof(telemetry_msg),
        "TELEMETRY:{"
        "\"uptime\":%lu,"
        "\"imu_status\":%d,"
        "\"imu_errors\":%d,"
        "\"battery\":%.2f,"
        "\"mode\":%d,"
        "\"temperature\":%d,"
        "\"commands\":%lu,"
        "\"packets\":%lu"
        "}\r\n",
        telemetry.uptime_ms,
        telemetry.imu_status,
        telemetry.imu_error_count,
        telemetry.battery_voltage,
        telemetry.system_mode,
        telemetry.temperature,
        telemetry.command_count,
        telemetry.telemetry_count
    );
    
    HAL_UART_Transmit(&huart1, (uint8_t*)telemetry_msg, strlen(telemetry_msg), HAL_MAX_DELAY);
}

// Send current IMU data
void send_imu_data(void) {
    // Read IMU (using your existing code)
    uint8_t g[6], a[6];
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, OUTX_L_G, I2C_MEMADD_SIZE_8BIT, g, 6, 50);
    if (status != HAL_OK) {
        telemetry.imu_error_count++;
        send_response("ERROR:IMU_READ_FAILED");
        return;
    }
    
    status = HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_ADDR, OUTX_L_XL, I2C_MEMADD_SIZE_8BIT, a, 6, 50);
    if (status != HAL_OK) {
        telemetry.imu_error_count++;
        send_response("ERROR:IMU_READ_FAILED");
        return;
    }
    
    // Process data (using your existing code)
    int16_t ax = (int16_t)(a[1] << 8 | a[0]);
    int16_t ay = (int16_t)(a[3] << 8 | a[2]);
    int16_t az = (int16_t)(a[5] << 8 | a[4]);
    
    int16_t gx = (int16_t)(g[1] << 8 | g[0]);
    int16_t gy = (int16_t)(g[3] << 8 | g[2]);
    int16_t gz = (int16_t)(g[5] << 8 | g[4]);
    
    // Calculate tilt
    int32_t tilt_x_int = ((int32_t)ay * 100) / 300;
    int32_t tilt_y_int = ((int32_t)ax * 100) / 300;
    
    char imu_msg[256];
    snprintf(imu_msg, sizeof(imu_msg),
        "IMU_DATA:{"
        "\"tilt_x\":%d.%02d,"
        "\"tilt_y\":%d.%02d,"
        "\"accel\":[%d,%d,%d],"
        "\"gyro\":[%d,%d,%d],"
        "\"timestamp\":%lu"
        "}\r\n",
        (int)(tilt_x_int/100), (int)(abs(tilt_x_int)%100),
        (int)(tilt_y_int/100), (int)(abs(tilt_y_int)%100),
        ax, ay, az,
        gx, gy, gz,
        HAL_GetTick()
    );
    
    HAL_UART_Transmit(&huart1, (uint8_t*)imu_msg, strlen(imu_msg), HAL_MAX_DELAY);
}

// Main loop integration
void main_loop_with_commands(void) {
    // Enable UART receive interrupt
    uint8_t dummy;
    HAL_UART_Receive_IT(&huart1, &dummy, 1);
    
    while (1) {
        // Update telemetry
        telemetry.uptime_ms = HAL_GetTick();
        
        // Handle different modes
        switch (telemetry.system_mode) {
            case MODE_IMU_STREAMING:
                // Send IMU data periodically
                send_imu_data();
                HAL_Delay(100);  // 10 Hz
                break;
                
            case MODE_SERVO_CONTROL:
                // Handle servo control
                break;
                
            case MODE_DIAGNOSTIC:
                // Send telemetry periodically
                send_telemetry();
                HAL_Delay(1000);  // 1 Hz
                break;
                
            default:
                HAL_Delay(100);
                break;
        }
    }
}