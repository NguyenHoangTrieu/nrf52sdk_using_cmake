#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

#if LEDS_NUMBER <= 2
#error "Board is not equipped with enough amount of LEDs"
#endif

#define TASK_DELAY        2000           /**< Task delay. Delays a LED0 task for 200 ms */
#define TIMER_PERIOD      1000          /**< Timer period. LED1 timer will expire after 1000 ms */

#define LED1_PIN NRF_GPIO_PIN_MAP(0, 13)  // P0.13
#define LED2_PIN NRF_GPIO_PIN_MAP(0, 14)  // P0.14
#define LED3_PIN NRF_GPIO_PIN_MAP(0, 15)  // P0.15
#define LED4_PIN NRF_GPIO_PIN_MAP(0, 17)  // P0.16

TaskHandle_t  led_toggle_task_handle;   /**< Reference to LED0 toggling FreeRTOS task. */
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */

/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
static void led_toggle_task_function (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    while (true)
    {


        /* Delay a task for a given number of ticks */
        nrf_gpio_pin_toggle(LED1_PIN);
        nrf_gpio_pin_toggle(LED3_PIN);
        vTaskDelay(TASK_DELAY);

        /* Tasks must be implemented to never return... */
    }
}

/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (void * pvParameter)
{
    UNUSED_PARAMETER(pvParameter);
    nrf_gpio_pin_toggle(LED2_PIN);
}

int main(void)
{
    ret_code_t err_code;
    /* Initialize clock driver for better time accuracy in FREERTOS */
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    
    /* Configure LED-pins as outputs */
    nrf_gpio_cfg_output(LED1_PIN);
    nrf_gpio_cfg_output(LED2_PIN);
    nrf_gpio_cfg_output(LED3_PIN);
    nrf_gpio_cfg_output(LED4_PIN);

    /* Create task for LED0 blinking with priority set to 2 */
    UNUSED_VARIABLE(xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_STACK_SIZE + 200, NULL, 2, &led_toggle_task_handle));

    /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreate( "LED1", TIMER_PERIOD, pdTRUE, NULL, led_toggle_timer_callback);
    UNUSED_VARIABLE(xTimerStart(led_toggle_timer_handle, 0));

    /* Activate deep sleep mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    while (true)
    {
        /* FreeRTOS should not be here... FreeRTOS goes back to the start of stack
         * in vTaskStartScheduler function. */
    }
}

// // Edge Impulse porting layer functions

// void ei_printf(const char *format, ...) {
//     char buffer[256];
//     va_list args;
//     va_start(args, format);
//     vsnprintf(buffer, sizeof(buffer), format, args);
//     va_end(args);

//     // Non-blocking with timeout
//     for (const char* p = buffer; *p != '\0'; p++) {
//         uint32_t timeout = 5; // 5ms timeout
//         while (app_uart_put((uint8_t)(*p)) != NRF_SUCCESS && timeout-- > 0) {
//             vTaskDelay(pdMS_TO_TICKS(1));
//         }
//         if (timeout == 0) break;
//     }
// }


// void ei_printf_float(float f) {
//     ei_printf("%.6f", f);
// }

// void *ei_malloc(size_t size) {
//     return pvPortMalloc(size);
// }

// void *ei_calloc(size_t nitems, size_t size) {
//     void *ptr = pvPortMalloc(nitems * size);
//     if (ptr) {
//         memset(ptr, 0, nitems * size);
//     }
//     return ptr;
// }

// void ei_free(void *ptr) {
//     vPortFree(ptr);
// }

// uint64_t ei_read_timer_ms(void) {
//     return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
// }

// uint64_t ei_read_timer_us(void) {
//     return (uint64_t)(xTaskGetTickCount() * portTICK_PERIOD_MS * 1000);
// }

// EI_IMPULSE_ERROR ei_sleep(int32_t time_ms) {
//     if (time_ms <= 0) {
//         taskYIELD();
//         return EI_IMPULSE_OK;
//     }
//     vTaskDelay(pdMS_TO_TICKS((uint32_t)time_ms));
//     return EI_IMPULSE_OK;
// }

// __attribute__((weak)) EI_IMPULSE_ERROR ei_run_impulse_check_canceled(void) {
//     return EI_IMPULSE_OK;
// }