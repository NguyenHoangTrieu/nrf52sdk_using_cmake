/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define MAX_TEST_DATA_BYTES     (15U)                /**< Maximum number of test bytes to be used for TX and RX. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define LED1_PIN NRF_GPIO_PIN_MAP(0, 14)            /**< Pin number for LED1. */

/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

// #include "nrf_nvic.h"
// void reset_to_uf2(void) {
//   NRF_POWER->GPREGRET = 0x57; // 0xA8 OTA, 0x4e Serial
//   NVIC_SystemReset();         // or sd_nvic_SystemReset();
// }

/**
 * @brief UART error event handler.
 * @param[in] p_event Pointer to UART event structure.
 */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**
 * @brief FreeRTOS task for transmitting data via UART.
 * @param[in] pvParameters Task parameters (unused).
 */
void uart_tx_task(void * pvParameters)
{
    const TickType_t delay_ticks = pdMS_TO_TICKS(1000);  // 100 ms delay
    static uint32_t counter = 0;
    char msg_buffer[64];
    
    UNUSED_PARAMETER(pvParameters);
    
    while (1)
    {
        // Create message with incrementing counter
        snprintf(msg_buffer, sizeof(msg_buffer), "FreeRTOS UART Task - Count: %lu\r\n", counter);
        
        // Send each character in the string
        for (const char * p = msg_buffer; *p != '\0'; p++)
        {
            // Retry until UART is ready to transmit
            while (app_uart_put((uint8_t)(*p)) != NRF_SUCCESS) 
            {
                vTaskDelay(pdMS_TO_TICKS(1)); // Short retry delay
            }
        }
        
        counter++;
        nrf_gpio_pin_toggle(LED1_PIN);
        vTaskDelay(delay_ticks); // Wait for 1 second
    }
}

/**
 * @brief FreeRTOS task for handling UART RX data (echo back functionality).
 * @param[in] pvParameters Task parameters (unused).
 */
void uart_rx_task(void * pvParameters)
{
    uint8_t rx_data;
    
    UNUSED_PARAMETER(pvParameters);
    
    while (1)
    {
        // Wait for incoming data
        if (app_uart_get(&rx_data) == NRF_SUCCESS)
        {
            // Echo back the received character
            while (app_uart_put(rx_data) != NRF_SUCCESS)
            {
                vTaskDelay(pdMS_TO_TICKS(1)); // Short retry delay
            }
            
            // If 'q' or 'Q' is received, send exit message
            if (rx_data == 'q' || rx_data == 'Q')
            {
                const char * exit_msg = "\r\nExit command received!\r\n";
                for (const char * p = exit_msg; *p != '\0'; p++)
                {
                    while (app_uart_put((uint8_t)(*p)) != NRF_SUCCESS)
                    {
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                }
            }
        }
        else
        {
            // No data available, short delay to prevent busy waiting
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

/**
 * @brief Application main function.
 */
int main(void)
{
    uint32_t err_code;

    // Initialize board support package for LEDs
    bsp_board_init(BSP_INIT_LEDS);
    nrf_gpio_cfg_output(LED1_PIN);

    // Configure UART communication parameters
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        UART_HWFC,
        false,
#if defined (UART_PRESENT)
        NRF_UART_BAUDRATE_115200
#else
        NRF_UARTE_BAUDRATE_115200
#endif
    };

    // Initialize UART with FIFO buffers
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);

    // Send startup message
    printf("\r\nFreeRTOS UART Example Started\r\n");

    // Create UART TX task with higher priority
    xTaskCreate(uart_tx_task, 
                "UART_TX",                                    // Task name
                configMINIMAL_STACK_SIZE + 200,               // Stack size
                NULL,                                         // Task parameters
                tskIDLE_PRIORITY + 2,                         // Task priority
                NULL);                                        // Task handle

    // Create UART RX task with lower priority
    xTaskCreate(uart_rx_task, 
                "UART_RX",                                    // Task name
                configMINIMAL_STACK_SIZE + 100,               // Stack size
                NULL,                                         // Task parameters
                tskIDLE_PRIORITY + 1,                         // Task priority
                NULL);                                        // Task handle

    // Enable deep sleep mode for power efficiency
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // The scheduler should never return, so this is an error condition
    while (true)
    {
        ASSERT(false);
    }
}
