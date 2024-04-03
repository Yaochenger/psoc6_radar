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


/* Header file includes */

#include <cybsp.h>
#include "cy_retarget_io.h"
#include "cyhal.h"


/* Header file for local tasks */
#include "radar_task.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* thread para */

#define THREAD_PRIORITY         10
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        20

/* thread handler */
static rt_thread_t ifxradar_task = RT_NULL;

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

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for example project that demonstrates presence
 * detection use case of radar. It sets up board support package, global
 * interrupts and UART. Two tasks are then created: one for presence
 * detection application, another for terminal UI to configure presence detection
 * parameters. Scheduler is then started.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  int
 *
 *******************************************************************************/

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define PRESENCE_DETECTION
//#define COUNTING_PERSONS

/* Timer resolution 1ms */
#define TICKS_PER_SECOND 1000

/* RADAR sensor SPI frequency */
#define SPI_FREQUENCY (10000000UL)

// Radar pins
/** Pin for the shared SPI CS Signal */
#define CYBSP_SPI_CS (P0_5)
/** Pin for the shared RESET Signal */
#define CYBSP_RESET (P11_4)
/** Pin for the shared LDO EN Signal */
#define CYBSP_LDO_EN (P5_0)
/** Pin for the shared IRQ Signal */
#define CYBSP_IRQ (P11_3)

/* Pin number designated for MISO PIN */

#define CYBSP_SPI_MISO (P0_3)
/* Pin number designated for MOSI PIN */
#define CYBSP_SPI_MOSI (P0_2)
/* Pin number designated for CLK PIN */
#define CYBSP_SPI_CLK (P0_4)

//leds
/* Pin number designated for LED RED */
#define LED_RED (P10_0)
/* Pin number designated for LED GREEN */
#define LED_GREEN (P10_1)

/* LED off */
#define LED_STATE_OFF (0U)
/* LED on */
#define LED_STATE_ON (1U)


#define UNUSED(x) (void)(x)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/

#if defined(COUNTING_PERSONS)
    static void radar_counter_callback(mtb_radar_sensing_context_t *context,
                                    mtb_radar_sensing_event_t event,
                                    mtb_radar_sensing_event_info_t *event_info,
                                    void *data);
#else
    static void radar_presence_sensing_callback(mtb_radar_sensing_context_t *context,
                                                mtb_radar_sensing_event_t event,
                                                mtb_radar_sensing_event_info_t *event_info,
                                                void *data);
#endif

/*******************************************************************************
 * Global Variables
 ********************************************************************************/

static volatile uint32_t ticks = 0;

static mtb_radar_sensing_mask_t application_type =
#if defined(COUNTING_PERSONS)
 MTB_RADAR_SENSING_MASK_COUNTER_EVENTS
#else
 MTB_RADAR_SENSING_MASK_PRESENCE_EVENTS
#endif
 ;

static mtb_radar_sensing_callback_t callback_function =
#if defined(COUNTING_PERSONS)
 radar_counter_callback
#else
 radar_presence_sensing_callback
#endif
 ;

static mtb_radar_sensing_context_t radar_context;

/*******************************************************************************
* Function Name: SysTick_Handler
********************************************************************************
* Summary:
* This is the interrupt handler function for the SysTick timer interrupt.
* It counts the time elapsed in milliseconds since the timer started.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/

/*******************************************************************************
* Function Name: sys_now
********************************************************************************
* Summary:
* Returns the current time in milliseconds.
*
* Parameters:
*  none
*
* Return:
*  the current time in milliseconds
*
*******************************************************************************/
__STATIC_INLINE uint64_t sys_now(void)
{
    return rt_tick_get();
}

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function. It does:
 *    1. Configure the board.
 *    2. Initializating the radar.
 *    3. Setting the callback function depending on the choosed application type: presence detection, counting persons.
 *    4. Enabling the radar sensing.
 *    5. Processing the radar events and printing the results on the console.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cyhal_spi_t mSPI;

    mtb_radar_sensing_hw_cfg_t hw_cfg = {.spi_cs = CYBSP_SPI_CS,
                                         .reset = CYBSP_RESET,
                                         .ldo_en = CYBSP_LDO_EN,
                                         .irq = CYBSP_IRQ,
                                         .spi = &mSPI};
    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize two LED ports and set LEDs' initial state to off */
    result = cyhal_gpio_init(LED_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cyhal_gpio_init(LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Activate radar reset pin */
    result = cyhal_gpio_init(hw_cfg.reset, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Enable LDO */
    result = cyhal_gpio_init(hw_cfg.ldo_en, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Enable IRQ pin */
    result = cyhal_gpio_init(hw_cfg.irq, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* CS handled manually */
    result = cyhal_gpio_init(hw_cfg.spi_cs, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Configure SPI interface */
    if (cyhal_spi_init(hw_cfg.spi,
                       CYBSP_SPI_MOSI,
                       CYBSP_SPI_MISO,
                       CYBSP_SPI_CLK,
                       NC,
                       NULL,
                       8,
                       CYHAL_SPI_MODE_00_MSB,
                       false) != CY_RSLT_SUCCESS)
        CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(CYBSP_SPI_MOSI), CYHAL_GET_PIN(CYBSP_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(CYBSP_SPI_MOSI), CYHAL_GET_PIN(CYBSP_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(CYBSP_SPI_CLK), CYHAL_GET_PIN(CYBSP_SPI_CLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(CYBSP_SPI_CLK), CYHAL_GET_PIN(CYBSP_SPI_CLK), CY_GPIO_DRIVE_1_8);

    /* Set the data rate to 25 Mbps */
    if (cyhal_spi_set_frequency(hw_cfg.spi, SPI_FREQUENCY) != CY_RSLT_SUCCESS)
        CY_ASSERT(result == CY_RSLT_SUCCESS);

    /* Radar initialization */
    result = mtb_radar_sensing_init(&radar_context, &hw_cfg, application_type);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("ERROR Radar Sensing init.\n\r");
        CY_ASSERT(0);
    }

    /* Register radar event callback */
    result = mtb_radar_sensing_register_callback(&radar_context, callback_function, NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("ERROR Radar Registering Callback.\n\r");
        CY_ASSERT(0);
    }

    /* Enable radar sensing */
    result = mtb_radar_sensing_enable(&radar_context);
    if (CY_RSLT_SUCCESS != result)
    {
        printf("ERROR Radar Enabling.\n\r");
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
//    printf("\x1b[2J\x1b[;H");

#if defined(COUNTING_PERSONS)
    printf("*************************** "
           "Radar Counting Persons Example "
           "*************************** \r\n\n");
#else
    printf("*************************** "
           "Radar Presence Example "
           "*************************** \r\n\n");
#endif

    for (;;)
    {
     /* Radar data processing */
        if (mtb_radar_sensing_process(&radar_context, sys_now()) != CY_RSLT_SUCCESS)
        {
            printf("cskit_radar_sensing_process error\n\r");
            CY_ASSERT(0);
        }

        rt_thread_mdelay(MTB_RADAR_SENSING_PROCESS_DELAY);
    }
}

/*******************************************************************************
* Function Name: radar_counter_callback
********************************************************************************
* Summary:
* Printing events for counter event and controlling the leds state.
*
* Parameters:
*  context: Pointer to context of radar
*   event: Event type
*   event_info: Pointer to structure holding the informations about the event
*   data: Pointer to data
*
* Return:
*  none
*
*******************************************************************************/
#if defined(COUNTING_PERSONS)
static void radar_counter_callback(mtb_radar_sensing_context_t *context,
                                   mtb_radar_sensing_event_t event,
                                   mtb_radar_sensing_event_info_t *event_info,
                                   void *data)
{
    UNUSED(context);
    UNUSED(data);

    switch (event)
    {
        case MTB_RADAR_SENSING_EVENT_COUNTER_IN:
            cyhal_gpio_write(LED_RED, LED_STATE_ON);
            cyhal_gpio_write(LED_GREEN, LED_STATE_OFF);
            printf("%.2f: Counter IN detected, IN: %d, OUT: %d\n\r",
                   (float)event_info->timestamp / 1000,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->in_count,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->out_count);
            break;
        case MTB_RADAR_SENSING_EVENT_COUNTER_OUT:
            cyhal_gpio_write(LED_RED, LED_STATE_OFF);
            cyhal_gpio_write(LED_GREEN, LED_STATE_ON);
            printf("%.2f: Counter OUT detected, IN: %d, OUT: %d\n\r",
                   (float)event_info->timestamp / 1000,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->in_count,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->out_count);
            break;
        case MTB_RADAR_SENSING_EVENT_COUNTER_OCCUPIED:
            cyhal_gpio_write(LED_RED, LED_STATE_ON);
            cyhal_gpio_write(LED_GREEN, LED_STATE_OFF);
            printf("%.2f: Counter occupied detected, IN: %d, OUT: %d\n\r",
                   (float)event_info->timestamp / 1000,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->in_count,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->out_count);
            break;
        case MTB_RADAR_SENSING_EVENT_COUNTER_FREE:
            cyhal_gpio_write(LED_RED, LED_STATE_OFF);
            cyhal_gpio_write(LED_GREEN, LED_STATE_ON);
            printf("%.2f: Counter free detected, IN: %d, OUT: %d\n\r",
                   (float)event_info->timestamp / 1000,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->in_count,
                   ((mtb_radar_sensing_counter_event_info_t *)event_info)->out_count);
            break;
        default:
        {
            printf("ERROR - Unknown event.\n\r");
        }
        break;
    }
}
#else

/*******************************************************************************
* Function Name: radar_presence_sensing_callback
********************************************************************************
* Summary:
* Printing events for presence event and controlling the leds state.
*
* Parameters:
*  context: Pointer to context of radar
*   event: Event type
*   event_info: Pointer to structure holding the informations about the event
*   data: Pointer to data
*
* Return:
*  none
*
*******************************************************************************/
static void radar_presence_sensing_callback(mtb_radar_sensing_context_t *context,
                                            mtb_radar_sensing_event_t event,
                                            mtb_radar_sensing_event_info_t *event_info,
                                            void *data)
{
    UNUSED(context);
    UNUSED(data);

    switch (event)
    {
        case MTB_RADAR_SENSING_EVENT_PRESENCE_IN:
        {
            cyhal_gpio_write(LED_RED, LED_STATE_ON);
            cyhal_gpio_write(LED_GREEN, LED_STATE_OFF);
            printf("%.3f: Presence IN %.2f-%.2f\n\r",
                   (float)event_info->timestamp / 1000,
                   (float)(((mtb_radar_sensing_presence_event_info_t *)event_info)->distance -
                           ((mtb_radar_sensing_presence_event_info_t *)event_info)->accuracy),
                   (float)(((mtb_radar_sensing_presence_event_info_t *)event_info)->distance +
                           ((mtb_radar_sensing_presence_event_info_t *)event_info)->accuracy));
        }
        break;
        case MTB_RADAR_SENSING_EVENT_PRESENCE_OUT:
        {
            cyhal_gpio_write(LED_RED, LED_STATE_OFF);
            cyhal_gpio_write(LED_GREEN, LED_STATE_ON);
            printf("%.3f: Presence OUT\n\r", (float)event_info->timestamp / 1000);
        }
        break;
        default:
        {
            printf("ERROR - Unknown event.\n\r");
        }
        break;
    }
}
#endif
