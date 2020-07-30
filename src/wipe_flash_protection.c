//*****************************************************************************
//
//! @file binary_counter.c
//!
//! @brief Example that displays the timer count on the LEDs.
//!
//! This example increments a variable on every timer interrupt. The global
//! variable is used to set the state of the LEDs. The example sleeps otherwise.
//!
//! SWO is configured in 1M baud, 8-n-1 mode.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2020, Ambiq Micro
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.4.2 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#define BRICK_KEY (0xA35C9B6D)

volatile uint32_t g_ui32TimerCount = 0;
uint64_t g_ui64BtnStatus = 0;

//*****************************************************************************
//
// Timer configuration.
//
//*****************************************************************************
am_hal_ctimer_config_t g_sTimer0 =
{
    // Don't link timers.
    0,

    // Set up Timer0A.
    (AM_HAL_CTIMER_FN_REPEAT |
     AM_HAL_CTIMER_INT_ENABLE    |
     AM_HAL_CTIMER_LFRC_32HZ),

    // No configuration for Timer0B.
    0,
};

//*****************************************************************************
//
// Function to initialize Timer A0 to interrupt every 1/4 second.
//
//*****************************************************************************
void
timerA0_init(void)
{
    uint32_t ui32Period;

    //
    // Enable the LFRC.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_LFRC);

    //
    // Set up timer A0.
    //
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
    am_hal_ctimer_config(0, &g_sTimer0);

    //
    // Set up timerA0 to 32Hz from LFRC divided to 1 second period.
    //
    ui32Period = 8;
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, ui32Period,
                             (ui32Period >> 1));

    //
    // Clear the timer Interrupt
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

} // timerA0_init()

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    //
    // Clear TimerA0 Interrupt (write to clear).
    //
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Increment count and set limit based on the number of LEDs available.
    //
    if (++g_ui32TimerCount >= (1 << AM_BSP_NUM_LEDS))
    {
        //
        // Reset the global.
        //
        g_ui32TimerCount = 0;
    }

} // am_ctimer_isr()

void BTN_init(void)
{
	am_hal_gpio_pin_config(AM_BSP_GPIO_BUTTON0, AM_HAL_GPIO_INPUT);
	am_hal_gpio_pin_config(AM_BSP_GPIO_BUTTON1, AM_HAL_GPIO_INPUT);
	am_hal_gpio_pin_config(AM_BSP_GPIO_BUTTON2, AM_HAL_GPIO_INPUT);
	//
    // Configure the GPIO/button interrupt polarity.
    //
    am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_BUTTON0, AM_HAL_GPIO_RISING);
	am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_BUTTON1, AM_HAL_GPIO_RISING);
	am_hal_gpio_int_polarity_bit_set(AM_BSP_GPIO_BUTTON2, AM_HAL_GPIO_RISING);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
	am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON1));
	am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON2));
	
    //
    // Enable the GPIO/button interrupt.
    //
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0));
	am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON1));
	am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON2));
	
    //
    // Enable GPIO interrupts to the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);


}


//*****************************************************************************
//
// GPIO ISR
//
//*****************************************************************************
void
am_gpio_isr(void)
{
	uint64_t btn_status = 0;
	//
	// Delay for debounce.
	//
	am_util_delay_ms(200);


	btn_status = am_hal_gpio_int_status_get(true);
	g_ui64BtnStatus = btn_status;

	//
	// Clear the GPIO Interrupt (write to clear).
	//
	am_hal_gpio_int_clear(btn_status);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
	uint32_t ui32Primask = 0;
	uint64_t ui64BtnStatus = 0;
	//
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set the default cache configuration
    //
    am_hal_cachectrl_enable(&am_hal_cachectrl_defaults);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_bsp_itm_string_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Binary Counter Example\n");

    //
    // TimerA0 init.
    //
    timerA0_init();

    //
    // Enable the timer Interrupt.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
    am_hal_interrupt_master_enable();

    //
    // Start timer A0
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
    //am_bsp_debug_printf_disable();
		
	BTN_init();

	am_util_stdio_printf("customer_info_signature status: %d\n",am_hal_flash_customer_info_signature_check());
	//am_util_stdio_printf("wipe_sram_enable status: %d\n",am_hal_flash_wipe_sram_enable_check());
	am_util_stdio_printf("wipe_flash_enable status: %d\n",am_hal_flash_wipe_flash_enable_check());
	am_util_stdio_printf("am_hal_flash_write_protect_check %d",am_hal_flash_write_protect_check((uint32_t *)0x80000,(uint32_t *)(0x80000+(16*1024)-1)));
	
    //am_hal_flash_wipe_flash_enable
    // Loop forever.
    //
    while (1)
    {
		ui64BtnStatus = g_ui64BtnStatus;

		if(g_ui64BtnStatus != 0)
		{
			//Entry critical section
			ui32Primask = am_hal_interrupt_master_disable();
			g_ui64BtnStatus = 0;
			//Exit critical section
			am_hal_interrupt_master_set(ui32Primask);
		}

		if(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON0) & ui64BtnStatus)
		{
			am_util_stdio_printf("btn_status=%x\n",ui64BtnStatus);
			//Entry critical section
			ui32Primask = am_hal_interrupt_master_disable();
			am_hal_flash_recovery(BRICK_KEY);
			//Exit critical section
			am_hal_interrupt_master_set(ui32Primask);
		}
		
		if(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON1) & ui64BtnStatus)
		{
			am_util_stdio_printf("btn_status=%x\n",ui64BtnStatus);
			//Entry critical section
			ui32Primask = am_hal_interrupt_master_disable();
			//am_hal_flash_info_signature_set();
			am_hal_flash_write_protect_set((uint32_t *)0x80000,(uint32_t *)(0x80000+(16*1024)-1));
			//Exit critical section
			am_hal_interrupt_master_set(ui32Primask);
			
		}
		
		if(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BUTTON2) & ui64BtnStatus)
		{
			int32_t ret = 0;
			am_util_stdio_printf("btn_status=%x\n",ui64BtnStatus);
			//Entry critical section
			ui32Primask = am_hal_interrupt_master_disable();
			ret = am_hal_flash_wipe_flash_enable();
			//Exit critical section
			am_hal_interrupt_master_set(ui32Primask);
			am_util_stdio_printf("ret=%x\n",ret);
			
		}
			
		//
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        //
        // Set the LEDs.
        //
        am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS,
                         g_ui32TimerCount);
    }
} // main()
