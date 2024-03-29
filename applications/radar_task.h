/******************************************************************************
** File Name:   radar_task.h
**
** Description: This file contains the function prototypes and constants used
**   in radar_task.c.
**
** Related Document: See README.md
**
** ===========================================================================
** Copyright (C) 2022 Infineon Technologies AG. All rights reserved.
** ===========================================================================
**
** ===========================================================================
** Infineon Technologies AG (INFINEON) is supplying this file for use
** exclusively with Infineon's sensor products. This file can be freely
** distributed within development tools and software supporting such
** products.
**
** THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
** OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
** MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
** INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT,
** INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON
** WHATSOEVER.
** ===========================================================================
*/

#pragma once

/* Header file includes */
#include "cycfg.h"
//#include "cyabs_rtos.h"
#include "mtb_radar_sensing.h"
//#include "rtthread.h"
#include "rtthread.h"

/* Pin number designated for LED RED */
#define LED_RGB_RED (P10_0)
/* Pin number designated for LED GREEN */
#define LED_RGB_GREEN (P10_1)
/* Pin number designated for LED BLUE */
#define LED_RGB_BLUE (P10_2)

/* LED off */
#define LED_STATE_OFF (0U)
/* LED on */
#define LED_STATE_ON (1U)

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Name of the radar task */
#define RADAR_TASK_NAME "RADAR PRESENCE TASK"
/* Stack size for the radar task */
#define RADAR_TASK_STACK_SIZE (4096)
/* Priority number for the radar task */
#define RADAR_TASK_PRIORITY (20)

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
extern mtb_radar_sensing_context_t sensing_context;

/*******************************************************************************
 * Functions
 *******************************************************************************/
void radar_task(void* arg);
void radar_presence_task_set_mute(bool mute);

/* USER ADD */

#define TARGET_CYSBSYSKIT_DEV_01
