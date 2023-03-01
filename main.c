/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the RDK4_CapSense_Buttons
*              Application for ModusToolbox.
*
* Related Document: See README.md
*
*
*  Created on: 2023-02-28
*  Company: Rutronik Elektronische Bauelemente GmbH
*  Address: Jonavos g. 30, Kaunas 44262, Lithuania
*  Author: GDR
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*
* Rutronik Elektronische Bauelemente GmbH Disclaimer: The evaluation board
* including the software is for testing purposes only and,
* because it has limited functions and limited resilience, is not suitable
* for permanent use under real conditions. If the evaluation board is
* nevertheless used under real conditions, this is done at one’s responsibility;
* any liability of Rutronik is insofar excluded
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cycfg_capsense.h"

#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#define EZI2C_INTR_PRIORITY              (2u)
#define BUTTON_DELAY_TIMES          	 (3u)

typedef struct capsense_data
{
    _Bool csb1_status;
    _Bool csb2_status;
    _Bool csb3_status;
}capsense_data_t;

cy_stc_scb_ezi2c_context_t ezi2c_context;
capsense_data_t cbuttons =
		{
				.csb1_status = false,
				.csb2_status = false,
				.csb3_status = false
		};

static void initialize_capsense(void);
static void capsense_msc1_isr(void);
static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);
static void handle_error(void);
static void process_touch(void);

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize the User LEDs */
    result = cyhal_gpio_init(USER_LED_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(USER_LED_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    result |= cyhal_gpio_init(USER_LED_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    if (result != CY_RSLT_SUCCESS)
    {handle_error();}
    cyhal_gpio_write((cyhal_gpio_t)USER_LED_RED, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)USER_LED_GREEN, CYBSP_LED_STATE_OFF);
    cyhal_gpio_write((cyhal_gpio_t)USER_LED_BLUE, CYBSP_LED_STATE_OFF);

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    /* Initialize MSC CapSense */
    initialize_capsense();

    /*Enable debug output via KitProg UART*/
    result = cy_retarget_io_init( KITPROG_TX, KITPROG_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("\x1b[2J\x1b[;H");
    printf("RDK4 CapSense CSX Buttons Example.\r\n");
    printf("Touch the CSB1, CSB2, CSB3 sensors on the board.\r\n");

    /* Start the first scan */
    Cy_CapSense_ScanAllSlots(&cy_capsense_context);

    for (;;)
    {
    	if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Process all widgets */
            Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

            /* Process touch input */
            process_touch();

            /* Establishes synchronized operation between the CapSense
             * middleware and the CapSense Tuner tool.
             */
            Cy_CapSense_RunTuner(&cy_capsense_context);

            /* Initiate next scan */
            Cy_CapSense_ScanAllSlots(&cy_capsense_context);
        }
    }
}

/*******************************************************************************
* Function Name: capsense_msc1_isr
********************************************************************************
* Summary:
*  Wrapper function for handling interrupts from CapSense MSC0 block.
*
*******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}

/*******************************************************************************
* Function Name: initialize_capsense_tuner
********************************************************************************
* Summary:
* EZI2C module to communicate with the CapSense Tuner tool.
*
*******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = ARD_I2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(ARD_I2C_HW, &ARD_I2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
    	CY_ASSERT(0);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(ARD_I2C_HW, (uint8_t *)&cy_capsense_tuner,
                            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
                            &ezi2c_context);

    Cy_SCB_EZI2C_Enable(ARD_I2C_HW);
}

/*******************************************************************************
* Function Name: initialize_capsense
********************************************************************************
* Summary:
*  This function initializes the CapSense and configures the CapSense
*  interrupt.
*
*******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC1_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);

        /* Initialize the CapSense firmware modules. */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}

/*******************************************************************************
* Function Name: process_touch
********************************************************************************
* Summary:
*  Gets the details of touch position detected, processes the touch input
*  and updates the LED status.
*
*******************************************************************************/
static void process_touch(void)
{
    uint32_t button1_status;
    uint32_t button2_status;
    uint32_t button3_status;

    static uint32_t cnt_1 = 0;
    static _Bool rel_1 = true;
    static uint32_t cnt_2 = 0;
    static _Bool rel_2 = true;
    static uint32_t cnt_3 = 0;
    static _Bool rel_3 = true;

    /* Get button 1 status */
    button1_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_CSB1_WDGT_ID,CY_CAPSENSE_CSB1_SNS0_ID,&cy_capsense_context);

    /* Get button 2 status */
    button2_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_CSB2_WDGT_ID,CY_CAPSENSE_CSB2_SNS0_ID,&cy_capsense_context);

    /* Get button 3 status */
    button3_status = Cy_CapSense_IsSensorActive(CY_CAPSENSE_CSB3_WDGT_ID,CY_CAPSENSE_CSB3_SNS0_ID,&cy_capsense_context);

    /* Detect new touch on Button1 */
    if(button1_status)
    {
    	if(cnt_1 == BUTTON_DELAY_TIMES)
    	{
    		cnt_1 = 0;
    		if(rel_1 == true)
    		{
    			cbuttons.csb1_status = !cbuttons.csb1_status;
    			rel_1 = false;

    			if(cbuttons.csb1_status)
    			{
    				printf("CSB1 activated.\r\n");
    				cyhal_gpio_write((cyhal_gpio_t)USER_LED_RED, CYBSP_LED_STATE_ON);
    			}
    			else
    			{
    				printf("CSB1 deactivated.\r\n");
    				cyhal_gpio_write((cyhal_gpio_t)USER_LED_RED, CYBSP_LED_STATE_OFF);
    			}
    		}
    	}
    	cnt_1++;
    }
    else
    {
    	rel_1 = true;
    	cnt_1 = 0;
    }

    /* Detect new touch on Button2 */
    if(button2_status)
    {
    	if(cnt_2 == BUTTON_DELAY_TIMES)
    	{
    		cnt_2 = 0;
    		if(rel_2 == true)
    		{
    			cbuttons.csb2_status = !cbuttons.csb2_status;
    			rel_2 = false;

    			if(cbuttons.csb2_status)
    			{
    				printf("CSB2 activated.\r\n");
    				cyhal_gpio_write((cyhal_gpio_t)USER_LED_GREEN, CYBSP_LED_STATE_ON);
    			}
    			else
    			{
    				printf("CSB2 deactivated.\r\n");
    				cyhal_gpio_write((cyhal_gpio_t)USER_LED_GREEN, CYBSP_LED_STATE_OFF);
    			}
    		}
    	}
    	cnt_2++;
    }
    else
    {
    	rel_2 = true;
    	cnt_2 = 0;
    }

    /* Detect new touch on Button3 */
    if(button3_status)
    {
    	if(cnt_3 == BUTTON_DELAY_TIMES)
    	{
    		cnt_3 = 0;
    		if(rel_3 == true)
    		{
    			cbuttons.csb3_status = !cbuttons.csb3_status;
    			rel_3 = false;

    			if(cbuttons.csb3_status)
    			{
    				printf("CSB3 activated.\r\n");
    				cyhal_gpio_write((cyhal_gpio_t)USER_LED_BLUE, CYBSP_LED_STATE_ON);
    			}
    			else
    			{
    				printf("CSB3 deactivated.\r\n");
    				cyhal_gpio_write((cyhal_gpio_t)USER_LED_BLUE, CYBSP_LED_STATE_OFF);
    			}
    		}
    	}
    	cnt_3++;
    }
    else
    {
    	rel_3 = true;
    	cnt_3 = 0;
    }
}

static void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: ezi2c_isr
********************************************************************************
* Summary:
* Wrapper function for handling interrupts from EZI2C block.
*
*******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(ARD_I2C_HW, &ezi2c_context);
}

/* [] END OF FILE */
