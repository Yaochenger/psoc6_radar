/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-06-29     Rbb666       first version
 */

#include <rtthread.h>
#include <rtdevice.h>

#include "drv_gpio.h"
/* USER_ADD */
#include "radar_task.h"

/*******************************************************************************
* Macros
*******************************************************************************/

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (9999)

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
/* Timer object used for blinking the LED */
cyhal_timer_t led_blink_timer;


#define LED_PIN     GET_PIN(0, 1)

/* USER_ADD */
static rt_thread_t ifxradar_task = RT_NULL;
int main(void)
{
    timer_init();
    cy_rslt_t result = CY_RSLT_SUCCESS;

    ifxradar_task = rt_thread_create(RADAR_TASK_NAME,
                                    radar_task,
                                    RT_NULL,
                                    RADAR_TASK_STACK_SIZE,
                                    RADAR_TASK_PRIORITY,
                                    5);
    if (ifxradar_task != RT_NULL)
        rt_thread_startup(ifxradar_task);


    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    for (;;)
    {
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(50);
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_thread_mdelay(50);
    }
}


void timer_init(void)
{
   cy_rslt_t result;

   const cyhal_timer_cfg_t led_blink_timer_cfg =
   {
       .compare_value = 0,                 /* Timer compare value, not used */
       .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
       .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
       .is_compare = false,                /* Don't use compare mode */
       .is_continuous = true,              /* Run timer indefinitely */
       .value = 0                          /* Initial value of counter */
   };

   /* Initialize the timer object. Does not use input pin ('pin' is NC) and
    * does not use a pre-configured clock source ('clk' is NULL). */
   result = cyhal_timer_init(&led_blink_timer, NC, NULL);

   /* timer init failed. Stop program execution */
   if (result != CY_RSLT_SUCCESS)
   {
       CY_ASSERT(0);
   }

   /* Configure timer period and operation mode such as count direction,
      duration */
   result = cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);
   if (result != CY_RSLT_SUCCESS)
   {
       CY_ASSERT(0);
   }

   /* Set the frequency of timer's clock source */
   result = cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);
   if (result != CY_RSLT_SUCCESS)
   {
       CY_ASSERT(0);
   }

   /* Assign the ISR to execute on timer interrupt */
   cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

   /* Set the event on which timer interrupt occurs and enable it */
   cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                            CYHAL_ISR_PRIORITY_DEFAULT, true);

   /* Start the timer with the configured settings */
   result = cyhal_timer_start(&led_blink_timer);
   if (result != CY_RSLT_SUCCESS)
   {
       CY_ASSERT(0);
   }
}

/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
   (void) callback_arg;
   (void) event;

#ifdef TARGET_CYSBSYSKIT_DEV_01
   /* Invert the USER LED state */
//    cyhal_gpio_toggle(CYBSP_USER_LED);
#endif
}

