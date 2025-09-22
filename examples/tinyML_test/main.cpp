/**
 * @file main.cpp
 * @brief nRF52840 Edge Impulse Accelerometer Classification Example
 *
 * This example demonstrates how to:
 * - Initialize MPU6050 accelerometer via I2C
 * - Collect accelerometer data continuously
 * - Run Edge Impulse inference on motion data
 * - Display classification results via UART
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// nRF SDK includes
#include "boards.h"
#include "bsp.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"
#include "nordic_common.h"

// Edge Impulse includes
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "edge-impulse-sdk/dsp/numpy.hpp"

extern "C" {
#include "mpu6050_driver.h"
}

// Configuration
#define UART_TX_BUF_SIZE        256
#define UART_RX_BUF_SIZE        256
#define INFERENCE_TASK_STACK_SIZE 2048  // Increased for memory safety
#define SENSOR_TASK_STACK_SIZE  1024
#define BLINK_TASK_STACK_SIZE   512
#define UART_RX_TASK_STACK_SIZE 512
#define INFERENCE_TASK_PRIORITY 2
#define SENSOR_TASK_PRIORITY    3
#define BLINK_TASK_PRIORITY     4
#define UART_RX_TASK_PRIORITY   1

// MPU6050 configuration - MATCH EDGE IMPULSE PROJECT
#define MPU6050_ADDR            0x68
#define SAMPLE_FREQ_HZ          EI_CLASSIFIER_FREQUENCY  // Use EI freq
#define WINDOW_SIZE_MS          (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * 1000 / (EI_CLASSIFIER_FREQUENCY * 3))  // Calculate from EI
#define INPUT_FRAME_SIZE        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // Use EI size

// UART pin configuration for nRF52840 DK
#define RX_PIN_NUMBER           8
#define TX_PIN_NUMBER           6
#define CTS_PIN_NUMBER          7
#define RTS_PIN_NUMBER          5
#define LED1_PIN                NRF_GPIO_PIN_MAP(0, 13) // P0.13
#define LED2_PIN                NRF_GPIO_PIN_MAP(0, 14) // P0.14
#define LED3_PIN                NRF_GPIO_PIN_MAP(0, 15) // P0.15

// Add Static allocation override for memory leak fix
// Static buffers to replace dynamic allocation in EI SDK
#define EI_STATIC_FEATURES_BUFFER_SIZE 8192     // 8KB
#define EI_STATIC_MATRIX_BUFFER_SIZE   8192     // 8KB  
#define EI_STATIC_GENERAL_BUFFER_SIZE  4096     // 4KB

static uint8_t ei_features_buffer[EI_STATIC_FEATURES_BUFFER_SIZE] __attribute__((aligned(16)));
static uint8_t ei_matrix_buffer[EI_STATIC_MATRIX_BUFFER_SIZE] __attribute__((aligned(16)));  
static uint8_t ei_general_buffer[EI_STATIC_GENERAL_BUFFER_SIZE] __attribute__((aligned(16)));

static volatile size_t ei_features_used = 0;
static volatile size_t ei_matrix_used = 0;
static volatile size_t ei_general_used = 0;

// Track total allocations for debugging
static volatile uint32_t total_ei_allocations = 0;
static volatile uint32_t total_ei_frees = 0;
//================================================================

// Global variables
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static QueueHandle_t sensor_data_queue;
static TaskHandle_t inference_task_handle;
static TaskHandle_t sensor_task_handle;
static TaskHandle_t blink_led_task_handle;
static TaskHandle_t uart_rx_task_handle;
static QueueHandle_t uart_rx_queue;  // Queue for UART RX data

// Sensor data structure
typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    uint32_t timestamp_ms;
} sensor_data_t;

// Buffer for inference - use EI size
static float inference_buffer[INPUT_FRAME_SIZE];
static size_t inference_buffer_idx = 0;

/**
 * @brief Custom UART print function with retry
 */
static void print_uart(const char* format, ...) {
    char msg_buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(msg_buffer, sizeof(msg_buffer), format, args);
    va_end(args);
    if (len >= (int)sizeof(msg_buffer)) {
        len = sizeof(msg_buffer) - 1;
    }
    msg_buffer[len] = '\0';
    for (const char* p = msg_buffer; *p != '\0'; p++) {
        uint32_t timeout = 10; // 10ms timeout per character
        while (app_uart_put((uint8_t)(*p)) != NRF_SUCCESS && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (timeout == 0) break; // Prevent blocking
    }
}

/**
 * @brief UART event handler
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static char rx_buffer[32];
    static uint8_t rx_index = 0;
    uint8_t cr;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            while (app_uart_get(&cr) == NRF_SUCCESS) {
                if (cr == '\r' || cr == '\n') {
                    if (rx_index > 0) {
                        rx_buffer[rx_index] = '\0';
                        xQueueSend(uart_rx_queue, rx_buffer, portMAX_DELAY);
                        rx_index = 0;
                    }
                } else if (rx_index < sizeof(rx_buffer) - 1) {
                    rx_buffer[rx_index++] = cr;
                }
            }
            break;
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize UART
 */
static void uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params = {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief TWI initialization
 */
static void twi_init(void)
{
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&m_twi);
}

/**
 * @brief Edge Impulse signal callback
 */
static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = inference_buffer[offset + i];
    }
    return EIDSP_OK;
}

/**
 * @brief Print inference result (fixed float printing)
 */
static void print_inference_result(ei_impulse_result_t* result)
{
    print_uart("Timing: DSP %d ms, Classification %d ms, Anomaly %d ms\r\n",
               result->timing.dsp,
               result->timing.classification,
               result->timing.anomaly);
    print_uart("Predictions:\r\n");
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        print_uart("  %s: %.5f\r\n",
                   result->classification[ix].label,
                   result->classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    print_uart("Anomaly score: %.3f\r\n", result->anomaly);
#endif
}

/**
 * @brief Process UART commands for fake data
 */
static void process_uart_command(char* cmd) {
    if (strncmp(cmd, "fake_static", 11) == 0) {
        mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_STATIC);
        print_uart("Switched to static fake data\r\n");
    } else if (strncmp(cmd, "fake_sine", 9) == 0) {
        mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_SINE);
        print_uart("Switched to sine fake data\r\n");
    } else if (strncmp(cmd, "fake_shake", 10) == 0) {
        mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_SHAKE);
        print_uart("Switched to shake fake data\r\n");
    } else if (strncmp(cmd, "fake_tilt", 9) == 0) {
        mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_TILT);
        print_uart("Switched to tilt fake data\r\n");
    } else if (strncmp(cmd, "fake_circle", 11) == 0) {
        mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_CIRCLE);
        print_uart("Switched to circle fake data\r\n");
    } else if (strncmp(cmd, "fake_off", 8) == 0) {
        mpu6050_disable_fake_data();
        print_uart("Fake data disabled\r\n");
    } else if (strncmp(cmd, "fake_custom", 11) == 0) {
        // Example: set custom values (parse from cmd if needed)
        mpu6050_set_custom_fake_data(0.5f, -0.3f, 1.2f, 45.0f, -30.0f, 15.0f);
        mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_STATIC);  // Use static for custom
        print_uart("Switched to custom fake data\r\n");
    }
}

/**
 * @brief UART RX task to handle commands
 */
static void uart_rx_task(void *pvParameter) {
    char cmd[32];
    while (1) {
        if (xQueueReceive(uart_rx_queue, cmd, portMAX_DELAY) == pdTRUE) {
            process_uart_command(cmd);
        }
    }
}

/**
 * @brief Sensor data collection task
 */
static void sensor_task(void *pvParameter)
{
    sensor_data_t sensor_data;
    uint8_t use_mpu6050 = 0;  // Fixed: default to fake mode
    
    // Init MPU6050
    if (mpu6050_init(&m_twi, MPU6050_ADDR) == NRF_SUCCESS) {
        print_uart("MPU6050 initialized successfully at %d Hz\r\n", SAMPLE_FREQ_HZ);
        use_mpu6050 = 1;
    } else {
        print_uart("ERROR: Failed to initialize MPU6050\r\n");
        print_uart("Using fake data mode only\r\n");
    }
    
    // Enable fake data (default: shake pattern for testing)
    if (mpu6050_enable_fake_data(MPU6050_FAKE_PATTERN_SHAKE) == NRF_SUCCESS) {
        print_uart("Fake data enabled with SHAKE pattern\r\n");
    }
    
    while (1) {
        // Read accel data (uses fake if enabled, or real if MPU working)
        if (mpu6050_read_accel(&m_twi, MPU6050_ADDR,
                              &sensor_data.accel_x,
                              &sensor_data.accel_y,
                              &sensor_data.accel_z) == NRF_SUCCESS) {
            sensor_data.timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Send to queue
            if (xQueueSend(sensor_data_queue, &sensor_data, 0) != pdTRUE) {
                // Queue full - skip sample
            }
        }
        
        // Toggle LED2 to show sensor activity
        nrf_gpio_pin_toggle(LED2_PIN);
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_FREQ_HZ));
    }
}

/**
 * @brief Inference task
 */
static void inference_task(void *pvParameter)
{
    sensor_data_t sensor_data;
    ei_impulse_result_t result;
    signal_t signal;
    EI_IMPULSE_ERROR ei_error;
    static uint32_t inference_count = 0;
    
    // Declare external functions from ei_static_override.cpp
    extern void ei_static_reset_buffers(void);
    extern void ei_static_get_usage(size_t* features, size_t* matrix, size_t* general, 
                                   uint32_t* allocs, uint32_t* frees);
    
    print_uart("Edge Impulse inference task started with STATIC ALLOCATION\r\n");
    print_uart("Expected input size: %d elements\r\n", INPUT_FRAME_SIZE);
    
    // Init buffer
    memset(inference_buffer, 0, sizeof(inference_buffer));
    inference_buffer_idx = 0;
    
    // Initialize classifier once
    run_classifier_init();
    
    while (1) {
        // Receive data
        if (xQueueReceive(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(50)) == pdTRUE) {
            // Add to buffer
            inference_buffer[inference_buffer_idx++] = sensor_data.accel_x;
            inference_buffer[inference_buffer_idx++] = sensor_data.accel_y;
            inference_buffer[inference_buffer_idx++] = sensor_data.accel_z;
            
            // If full, run inference with static allocation
            if (inference_buffer_idx >= INPUT_FRAME_SIZE) {
                inference_count++;
                print_uart("\r\n=== Running Edge Impulse inference (#%u) ===\r\n", inference_count);
                print_uart("Free heap: %u bytes\r\n", xPortGetFreeHeapSize());
                
                // CRITICAL: Reset static buffers before each inference
                ei_static_reset_buffers();
                
                // Setup signal
                signal.total_length = INPUT_FRAME_SIZE;
                signal.get_data = &get_signal_data;
                
                // Run classifier with static allocation
                uint32_t start_time = xTaskGetTickCount();
                ei_error = run_classifier(&signal, &result, false);
                uint32_t inference_time = (xTaskGetTickCount() - start_time) * portTICK_PERIOD_MS;
                
                // Get buffer usage stats
                size_t features_used, matrix_used, general_used;
                uint32_t allocs, frees;
                ei_static_get_usage(&features_used, &matrix_used, &general_used, &allocs, &frees);
                print_uart("Static buffer usage: Features=%u, Matrix=%u, General=%u\r\n", 
                          features_used, matrix_used, general_used);
                print_uart("EI Allocations: %u malloc calls, %u free calls\r\n", allocs, frees);
                
                if (ei_error != EI_IMPULSE_OK) {
                    print_uart("ERROR: Failed to run classifier (%d)\r\n", ei_error);
                } else {
                    print_inference_result(&result);
                    print_uart("Inference time: %u ms\r\n", inference_time);
                    
                    // Find max confidence
                    float max_confidence = 0.0f;
                    const char* predicted_class = "unknown";
                    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                        if (result.classification[ix].value > max_confidence) {
                            max_confidence = result.classification[ix].value;
                            predicted_class = result.classification[ix].label;
                        }
                    }
                    print_uart(">>> PREDICTION: %s (%.1f%%)\r\n",
                               predicted_class, max_confidence * 100.0f);
                }
                
                // Reset buffer for next window
                inference_buffer_idx = 0;
                print_uart("=== Collecting next window ===\r\n");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/**
 * @brief LED blink task
 */
static void blink_led_task(void *pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (1) {
        nrf_gpio_pin_toggle(LED1_PIN);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief Main function
 */
int main(void)
{
    // Init GPIO
    nrf_gpio_cfg_output(LED1_PIN);
    nrf_gpio_cfg_output(LED2_PIN);
    nrf_gpio_cfg_output(LED3_PIN);
    
    // Init UART
    uart_init();
    print_uart("\r\n=== nRF52840 Edge Impulse Accelerometer Demo ===\r\n");
    
    // Init TWI
    twi_init();
    
    // Print EI info
    print_uart("Edge Impulse model info:\r\n");
    print_uart("  Input frame size: %d\r\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    print_uart("  Sample frequency: %d Hz\r\n", EI_CLASSIFIER_FREQUENCY);
    print_uart("  Label count: %d\r\n", EI_CLASSIFIER_LABEL_COUNT);
    
    // Create queues
    sensor_data_queue = xQueueCreate(16, sizeof(sensor_data_t));
    if (sensor_data_queue == NULL) {
        print_uart("ERROR: Failed to create sensor queue\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    uart_rx_queue = xQueueCreate(10, 32);  // For UART commands
    if (uart_rx_queue == NULL) {
        print_uart("ERROR: Failed to create UART RX queue\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    // Create tasks
    if (xTaskCreate(sensor_task, "SENSOR", SENSOR_TASK_STACK_SIZE, NULL,
                    SENSOR_TASK_PRIORITY, &sensor_task_handle) != pdPASS) {
        print_uart("ERROR: Failed to create sensor task\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    if (xTaskCreate(inference_task, "INFERENCE", INFERENCE_TASK_STACK_SIZE, NULL,
                    INFERENCE_TASK_PRIORITY, &inference_task_handle) != pdPASS) {
        print_uart("ERROR: Failed to create inference task\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    if (xTaskCreate(blink_led_task, "BLINK", BLINK_TASK_STACK_SIZE, NULL,
                    BLINK_TASK_PRIORITY, &blink_led_task_handle) != pdPASS) {
        print_uart("ERROR: Failed to create blink task\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    if (xTaskCreate(uart_rx_task, "UART_RX", UART_RX_TASK_STACK_SIZE, NULL,
                    UART_RX_TASK_PRIORITY, &uart_rx_task_handle) != pdPASS) {
        print_uart("ERROR: Failed to create UART RX task\r\n");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    
    print_uart("Tasks created. Starting scheduler...\r\n");
    
    // Start scheduler
    vTaskStartScheduler();
    
    print_uart("ERROR: Failed to start scheduler\r\n");
    
    // Never reach here
    while (1) {}
}

// Edge Impulse porting functions
void ei_printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    for (const char* p = buffer; *p != '\0'; p++) {
        uint32_t timeout = 5;
        while (app_uart_put((uint8_t)(*p)) != NRF_SUCCESS && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (timeout == 0) break;
    }
}

void ei_printf_float(float f) {
    ei_printf("%.6f", f);
}

uint64_t ei_read_timer_ms(void) {
    return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

uint64_t ei_read_timer_us(void) {
    return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000);
}

EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
    if (time_ms <= 0) {
        taskYIELD();
        return EI_IMPULSE_OK;
    }
    vTaskDelay(pdMS_TO_TICKS((uint32_t)time_ms));
    return EI_IMPULSE_OK;
}

__attribute__((weak)) EI_IMPULSE_ERROR ei_run_impulse_check_canceled(void) {
    return EI_IMPULSE_OK;
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    while(1) {
        nrf_gpio_pin_toggle(LED1_PIN);
        nrf_delay_ms(50); // Fast blink for stack overflow
    }
}

extern "C" void vApplicationMallocFailedHook(void)
{
    while(1) {
        nrf_gpio_pin_toggle(LED1_PIN);
        nrf_delay_ms(25); // Very fast blink for malloc fail
    }
}

// Override ei_malloc to use static buffers (REPLACE EXISTING)
void *ei_malloc(size_t size) {
    total_ei_allocations++;
    size = (size + 15) & ~15;
    
    // Try static buffers first (smaller now)
    if (ei_features_used + size <= sizeof(ei_features_buffer)) {
        void* ptr = &ei_features_buffer[ei_features_used];
        ei_features_used += size;
        return ptr;
    }
    
    if (ei_matrix_used + size <= sizeof(ei_matrix_buffer)) {
        void* ptr = &ei_matrix_buffer[ei_matrix_used];
        ei_matrix_used += size;
        return ptr;
    }
    
    if (ei_general_used + size <= sizeof(ei_general_buffer)) {
        void* ptr = &ei_general_buffer[ei_general_used];
        ei_general_used += size;
        return ptr;
    }
    
    // Fall back to heap (will be leaked, but prevents crash)
    print_uart("EI malloc fallback to heap: %u bytes\r\n", size);
    return pvPortMalloc(size);
}

// Override ei_calloc to use static buffers (REPLACE EXISTING)
void *ei_calloc(size_t nitems, size_t size) {
    size_t total = nitems * size;
    void* ptr = ei_malloc(total);
    if (ptr) {
        memset(ptr, 0, total);
    }
    return ptr;
}

// Override ei_free - static buffers don't need individual freeing (REPLACE EXISTING)
void ei_free(void *ptr) {
    total_ei_frees++;
    
    // Check if pointer is in our static buffers
    if (ptr >= (void*)ei_features_buffer && ptr < (void*)(ei_features_buffer + sizeof(ei_features_buffer))) {
        // Don't free static buffer memory
        return;
    }
    if (ptr >= (void*)ei_matrix_buffer && ptr < (void*)(ei_matrix_buffer + sizeof(ei_matrix_buffer))) {
        // Don't free static buffer memory
        return;
    }
    if (ptr >= (void*)ei_general_buffer && ptr < (void*)(ei_general_buffer + sizeof(ei_general_buffer))) {
        // Don't free static buffer memory
        return;
    }
    
    // If not from static buffers, free from heap
    if (ptr) {
        vPortFree(ptr);
    }
}

// Function to reset static buffer usage (call before each inference)
void ei_static_reset_buffers(void) {
    ei_features_used = 0;
    ei_matrix_used = 0;  
    ei_general_used = 0;
}

// Function to get buffer usage stats (for debugging)
void ei_static_get_usage(size_t* features, size_t* matrix, size_t* general, 
                        uint32_t* allocs, uint32_t* frees) {
    if (features) *features = ei_features_used;
    if (matrix) *matrix = ei_matrix_used;
    if (general) *general = ei_general_used;
    if (allocs) *allocs = total_ei_allocations;
    if (frees) *frees = total_ei_frees;
}