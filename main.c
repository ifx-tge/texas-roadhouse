/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4: CY8CPROTO-040T Demo
 *              code example for ModusToolbox
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/

#include "cy_pdl.h"
#include "cy_result.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"

/*******************************************************************************
 * User configurable Macros
 ********************************************************************************/

/* enable only one (1) physical interface below for ACTIVE PRO, adjust SCB0 accordingly */
#define ACTIVE_SPIM_ENABLED				(0u)
#define ACTIVE_GPIO_ENABLED				(0u)
#define ACTIVE_UART_ENABLED				(1u)

/* Comment this line out to remove all ACTIVE Debug output from your project */
#define ACTIVE_DEBUG_ON

/* Enables the Runtime measurement functionality used to for processing time measurement */
#define ENABLE_RUN_TIME_MEASUREMENT     (0u)

/* Enable this, if Tuner needs to be enabled */
#define ENABLE_TUNER                    (1u)

/* Enable PWM controlled LEDs */
#define ENABLE_PWM_LED                  (1u)

/* 128Hz (7.8125msec) Refresh rate in Active mode */
#define ACTIVE_MODE_REFRESH_RATE        (128u)

/* 32Hz (31.24msec) Refresh rate in Active-Low Refresh rate(ALR) mode */
#define ALR_MODE_REFRESH_RATE           (32u)

/* Timeout (10 seconds) to move from ACTIVE mode to ALR mode if there is no user activity */
#define ACTIVE_MODE_TIMEOUT_SEC         (10u)

/* Timeout (5 seconds) to move from ALR mode to WOT mode if there is no user activity */
#define ALR_MODE_TIMEOUT_SEC            (5u)

/* Active mode Scan time calculated in us ~= 87us */
#define ACTIVE_MODE_FRAME_SCAN_TIME     (87u)

/* Active mode Processing time in us ~= 92us with LEDs and Tuner disabled*/
#define ACTIVE_MODE_PROCESS_TIME        (92u)

/* ALR mode Scan time calculated in us ~= 87us */
#define ALR_MODE_FRAME_SCAN_TIME        (87u)

/* ALR mode Processing time in us ~= 92us with LEDs and Tuner disabled*/
#define ALR_MODE_PROCESS_TIME           (92u)

/*******************************************************************************
 * Macros
 ********************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY     (3u)
#define CY_ASSERT_FAILED                (0u)

/* EZI2C interrupt priority must be higher than CAPSENSE&trade; interrupt. */
#define EZI2C_INTR_PRIORITY             (2u)

#define ILO_FREQ                        (40000u)
#define TIME_IN_US                      (1000000u)

#define MINIMUM_TIMER                   (TIME_IN_US / ILO_FREQ)
#if ((TIME_IN_US / ACTIVE_MODE_REFRESH_RATE) > (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
#define ACTIVE_MODE_TIMER           (TIME_IN_US / ACTIVE_MODE_REFRESH_RATE - \
        (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
#elif
#define ACTIVE_MODE_TIMER           (MINIMUM_TIMER)
#endif

#if ((TIME_IN_US / ALR_MODE_REFRESH_RATE) > (ALR_MODE_FRAME_SCAN_TIME + ALR_MODE_PROCESS_TIME))
#define ALR_MODE_TIMER              (TIME_IN_US / ALR_MODE_REFRESH_RATE - \
        (ALR_MODE_FRAME_SCAN_TIME + ALR_MODE_PROCESS_TIME))
#elif
#define ALR_MODE_TIMER              (MINIMUM_TIMER)
#endif

#define ACTIVE_MODE_TIMEOUT             (ACTIVE_MODE_REFRESH_RATE * ACTIVE_MODE_TIMEOUT_SEC)

#define ALR_MODE_TIMEOUT                (ALR_MODE_REFRESH_RATE * ALR_MODE_TIMEOUT_SEC)

#define TIMEOUT_RESET                   (0u)

#if ENABLE_RUN_TIME_MEASUREMENT
#define SYS_TICK_INTERVAL           (0x00FFFFFF)
#define TIME_PER_TICK_IN_US         ((float)1/CY_CAPSENSE_CPU_CLK)*TIME_IN_US
#endif

#if ENABLE_PWM_LED
#define CYBSP_LED_OFF                   (0u)
#define CYBSP_LED_ON                    (1u)
#endif

#define LP_SLOT_START_ID                (0u)
#define LP_SLOT_NUMBER                  (1u)
/*****************************************************************************
 * Finite state machine states for device operating states
 *****************************************************************************/
typedef enum
{
    ACTIVE_MODE = 0x01u,    /* Active mode - All the sensors are scanned in this state
     * with highest refresh rate */
    ALR_MODE = 0x02u,       /* Active-Low Refresh Rate (ALR) mode - All the sensors are
     * scanned in this state with low refresh rate */
    WOT_MODE = 0x03u        /* Wake on Touch (WoT) mode - Low Power sensors are scanned
     * in this state with lowest refresh rate */
} APPLICATION_STATE;

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static void initialize_capsense(void);
static void capsense_msc0_isr(void);

static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);

#if ENABLE_RUN_TIME_MEASUREMENT
static void init_sys_tick();
static void start_runtime_measurement();
static uint32_t stop_runtime_measurement();
#endif

void led_control();

/* Deep Sleep Callback function */
void register_callback(void);
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
        cy_en_syspm_callback_mode_t mode);

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/

/* Variables holds the current low power state [ACTIVE, ALR or WOT] */
APPLICATION_STATE capsense_state;

cy_stc_scb_ezi2c_context_t ezi2c_context;

/* Callback parameters for custom, EzI2C */

/* Callback parameters for EzI2C */
cy_stc_syspm_callback_params_t ezi2cCallbackParams =
{
        .base       = SCB1,
        .context    = &ezi2c_context
};

/* Callback parameters for custom callback */
cy_stc_syspm_callback_params_t deepSleepCallBackParams = {
        .base       =  NULL,
        .context    =  NULL
};

/* Callback declaration for EzI2C Deep Sleep callback */
cy_stc_syspm_callback_t ezi2cCallback =
{
        .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &ezi2cCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
};

/* Callback declaration for Custom Deep Sleep callback */
cy_stc_syspm_callback_t deepSleepCb =
{
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &deepSleepCallBackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 2
};


uint32_t process_time=0;

#if ACTIVE_SPIM_ENABLED

/*******************************************************************************
 * SPIM Macros
 ******************************************************************************/
/* Initialization status */
#define INIT_SUCCESS            (0)
#define INIT_FAILURE            (1)

/* Element index in the packet */
#define PACKET_SOP_POS          (0UL)
#define PACKET_CMD_POS          (1UL)
#define PACKET_EOP_POS          (2UL)

/* TX Packet Head and Tail */
#define PACKET_SOP          (0x01UL)
#define PACKET_EOP          (0x17UL)

/* Assign SPI interrupt priority */
#define mSPI_INTR_PRIORITY  (3U)

/* Number of elements in the transmit buffer */
/* There are six elements - one for head, one for command and one for tail and then random data */
#define NUMBER_OF_ELEMENTS  (6UL)
#define SIZE_OF_ELEMENT     (1UL)
#define SIZE_OF_PACKET      (NUMBER_OF_ELEMENTS * SIZE_OF_ELEMENT)

/***************************************
* SPIM Function Prototypes
****************************************/
uint32_t initMaster(void);
cy_en_scb_spi_status_t sendPacket(uint8_t *, uint32_t);

/*******************************************************************************
 * SPIM Global Variables
 ******************************************************************************/
cy_stc_scb_spi_context_t mSPI_context;

#endif

#if ACTIVE_UART_ENABLED
cy_stc_scb_uart_context_t CYBSP_UART_context;
#endif

// ACTIVEPRO firmware tool stuff ->>>>

void ACTIVEValue( int channel, int value );          // Output a Value for this channel
void ACTIVEText( int channel, char *string );        // Output Text for this channel
void ACTIVEprintf( int channel, char *format, ... ); // printf-like function with variable argument list

// Define helpful macros for sending debug command messages
#define ACTIVELabel(channel,string)  ACTIVEText( (channel) , "?cmd=label&label=" string )
#define ACTIVEBeep()  ACTIVEText( 0 , "?cmd=beep" )
#define ACTIVEStop()  ACTIVEText( 0 , "?cmd=stop" )
#define ACTIVERestart()  ACTIVEText( 0 , "?cmd=restart" )

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE&trade;
 *  - initialize tuner communication
 *  - scan touch input continuously at 3 different power modes
 *  - LED for touch indication
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t capsense_state_timeout;
    uint32_t interruptStatus;

	#if ACTIVE_SPIM_ENABLED
    cy_en_scb_spi_status_t status;
    uint32_t cmd_send = CYBSP_LED_STATE_OFF;

    /* Buffer to hold command packet to be sent to the slave by the master */
    uint8_t  txBuffer[NUMBER_OF_ELEMENTS];
	#endif

    result = cybsp_init() ;

    #if ENABLE_RUN_TIME_MEASUREMENT
    init_sys_tick();
    #endif

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

	#if ACTIVE_SPIM_ENABLED
    /* Initialize the SPI Master */
    result = initMaster();

    /* Initialization failed. Stop program execution */
    if(result != INIT_SUCCESS)
    {
        CY_ASSERT(0);
    }
	#endif

	#if ACTIVE_UART_ENABLED
    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);
	#endif

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    #if ENABLE_PWM_LED
    /* Initialize PWM block */
    (void)Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);
    /* Enable the initialized PWM */
    Cy_TCPWM_Enable_Multiple(CYBSP_PWM_HW, CYBSP_PWM_MASK);
    /* Then start the PWM */
    Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_HW, CYBSP_PWM_MASK);
    #endif

    /* Register callbacks */
    register_callback();

    /* Define initial state of the device and the corresponding refresh rate*/
    capsense_state = ACTIVE_MODE;
    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;

    /* Initialize MSC CAPSENSE&trade; */
    initialize_capsense();

    /* Measures the actual ILO frequency and compensate MSCLP wake up timers */
    Cy_CapSense_IloCompensate(&cy_capsense_context);

    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);

    /* ACTIVE Pro Firmware Development / Debugging Tool */
    ACTIVERestart();

    for (;;)
    {
//    	ACTIVELabel(0x0u,"State");

    	ACTIVEValue(0x0u, capsense_state);

        switch(capsense_state)
        {
            case ACTIVE_MODE:

            	ACTIVEText(0x1u, "ACTIVE");

                Cy_CapSense_ScanAllSlots(&cy_capsense_context);

                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    #if ENABLE_PWM_LED
                	ACTIVEText(0x8u, "CpuSleep");
                    Cy_SysPm_CpuEnterSleep();
                    #else
                    ACTIVEText(0x8u, "CpuDeepSleep");
                    Cy_SysPm_CpuEnterDeepSleep();
                    #endif

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();
                }

                #if ENABLE_RUN_TIME_MEASUREMENT
                uint32_t active_processing_time=0;
                start_runtime_measurement();
                #endif

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                if(Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
                {
                    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;
                }
                else
                {
                    capsense_state_timeout--;

                    if(TIMEOUT_RESET == capsense_state_timeout)
                    {

                        capsense_state = ALR_MODE;
                        capsense_state_timeout = ALR_MODE_TIMEOUT;

                        /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                        Cy_CapSense_ConfigureMsclpTimer(ALR_MODE_TIMER, &cy_capsense_context);
                    }
                }

                #if ENABLE_RUN_TIME_MEASUREMENT
                active_processing_time=stop_runtime_measurement();
                #endif

                break;
                /* End of ACTIVE_MODE */

                /* Active Low Refresh-rate Mode */
            case ALR_MODE :
            	ACTIVEText(0x2u, "ALR");

                Cy_CapSense_ScanAllSlots(&cy_capsense_context);
                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    #if ENABLE_PWM_LED
                    Cy_SysPm_CpuEnterSleep();
                    #else
                    Cy_SysPm_CpuEnterDeepSleep();
                    #endif

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();
                }

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                #if ENABLE_RUN_TIME_MEASUREMENT
                uint32_t alr_processing_time=0;
                start_runtime_measurement();
                #endif

                Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                if(Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_MODE;
                    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;

                    #if ENABLE_PWM_LED
                    /* Initialize PWM block */
                    (void)Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);
                    /* Enable the initialized PWM */
                    Cy_TCPWM_Enable_Multiple(CYBSP_PWM_HW, CYBSP_PWM_MASK);
                    /* Then start the PWM */
                    Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_HW, CYBSP_PWM_MASK);
                    #endif

                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }
                else
                {
                    capsense_state_timeout--;

                    if(TIMEOUT_RESET == capsense_state_timeout)
                    {
                        capsense_state = WOT_MODE;
                    }
                }

#if ENABLE_RUN_TIME_MEASUREMENT
                alr_processing_time=stop_runtime_measurement();
#endif

                break;
                /* End of Active-Low Refresh Rate(ALR) mode */

                /* Wake On Touch Mode */
            case WOT_MODE :
            	ACTIVEText(0x3u, "WOT");

                /* Scanning only those low-power slots needed for the specific configuration*/
                Cy_CapSense_ScanLpSlots(LP_SLOT_START_ID, LP_SLOT_NUMBER, &cy_capsense_context);

                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
//                	ACTIVEText(0x4u, "IsBusy");
                    Cy_SysPm_CpuEnterDeepSleep();

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();
                }

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                if (Cy_CapSense_IsAnyLpWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_MODE;
                    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;

                    #if ENABLE_PWM_LED
                    /* Initialize PWM block */
                    (void)Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);
                    /* Enable the initialized PWM */
                    Cy_TCPWM_Enable_Multiple(CYBSP_PWM_HW, CYBSP_PWM_MASK);
                    /* Then start the PWM */
                    Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_HW, CYBSP_PWM_MASK);
                    #endif

                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }
                else
                {
                    capsense_state = ALR_MODE;
                    capsense_state_timeout = ALR_MODE_TIMEOUT;

                    /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ALR_MODE_TIMER, &cy_capsense_context);
                }

                break;
                /* End of "WAKE_ON_TOUCH_MODE" */

            default:
                /**  Unknown power mode state. Unexpected situation.  **/
                CY_ASSERT(CY_ASSERT_FAILED);
                break;
        }

        #if ENABLE_PWM_LED
        led_control();
        #endif

        #if ENABLE_TUNER
        /* Establishes synchronized communication with the CAPSENSE&trade; Tuner tool */
        Cy_CapSense_RunTuner(&cy_capsense_context);
        ACTIVEValue(0x0Au, cy_capsense_tuner.sensorContext[2u].diff); //CY_CAPSENSE_BUTTON0_WDGT_ID
        ACTIVEValue(0x0Bu, cy_capsense_tuner.sensorContext[3u].diff); //CY_CAPSENSE_BUTTON1_WDGT_ID
        #endif

		#if ACTIVE_SPIM_ENABLED
//        /* Toggle the current LED state */
//        cmd_send = (cmd_send == CYBSP_LED_STATE_OFF) ? CYBSP_LED_STATE_ON :
//                                                       CYBSP_LED_STATE_OFF;
//
//        /* Form the command + data packet */
//        txBuffer[PACKET_SOP_POS] = 0x7F;
//        txBuffer[PACKET_CMD_POS] = 0x3F;
//        txBuffer[PACKET_EOP_POS] = 0x40;
//        txBuffer[3UL] = 0x08; //
//        txBuffer[4UL] = 0x04; //
//        txBuffer[5UL] = 0x02; //
//
//        /* Send the packet to the slave */
//        status = sendPacket(txBuffer, SIZE_OF_PACKET);
//        if(status != CY_SCB_SPI_SUCCESS)
//        {
//            CY_ASSERT(0);
//        }
		#endif

		#if ACTIVE_GPIO_ENABLED
//        Cy_GPIO_Inv(ACTIVE_SPI_MOSI_PORT, ACTIVE_SPI_MOSI_PIN);
//        Cy_GPIO_Inv(ACTIVE_SPI_CLK_PORT, ACTIVE_SPI_CLK_PIN);
		#endif

		#if ACTIVE_UART_ENABLED
        /* Send a string over serial terminal */
//        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Hello world\r\n");
//        Cy_SCB_UART_Put(CYBSP_UART_HW, 0x7F);
		#endif

        /* Delay between commands */
//        Cy_SysLib_Delay(1000);

    }
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CAPSENSE&trade; and configures the CAPSENSE&trade;
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CAPSENSE&trade; interrupt configuration MSCLP 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
            .intrSrc = CY_MSCLP0_LP_IRQ,
            .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CAPSENSE&trade; interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CAPSENSE&trade; sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}

/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CAPSENSE&trade; MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSCLP0_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CAPSENSE&trade; Tuner tool.
 *
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
            .intrSrc = CYBSP_EZI2C_IRQ,
            .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CAPSENSE&trade; data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
            &ezi2c_context);

    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

}

/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from EZI2C block.
 *
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

#if ENABLE_RUN_TIME_MEASUREMENT
/*******************************************************************************
 * Function Name: init_sys_tick
 ********************************************************************************
 * Summary:
 *  initializes the system tick with highest possible value to start counting down.
 *
 *******************************************************************************/
static void init_sys_tick()
{
    Cy_SysTick_Init (CY_SYSTICK_CLOCK_SOURCE_CLK_CPU ,0x00FFFFFF);
}
#endif


#if ENABLE_RUN_TIME_MEASUREMENT
/*******************************************************************************
 * Function Name: start_runtime_measurement
 ********************************************************************************
 * Summary:
 *  Initializes the system tick counter by calling Cy_SysTick_Clear() API.
 *******************************************************************************/
static void start_runtime_measurement()
{
    Cy_SysTick_Clear();
}

/*******************************************************************************
 * Function Name: stop_runtime_measurement
 ********************************************************************************
 * Summary:
 *  Reads the system tick and converts to time in microseconds(us).
 *
 *  Returns:
 *  runtime - in microseconds(us)
 *******************************************************************************/

static uint32_t stop_runtime_measurement()
{
    uint32_t ticks;
    uint32_t runtime;
    ticks=Cy_SysTick_GetValue();
    ticks= SYS_TICK_INTERVAL - Cy_SysTick_GetValue();
    runtime=ticks*TIME_PER_TICK_IN_US;
    return runtime;
}
#endif

#if ENABLE_PWM_LED
/*******************************************************************************
 * Function Name: led_control
 ********************************************************************************
 * Summary:
 *  Control LED2 in the kit to show the CSD button and CSX button status:
 *    No touch on either- LED2 == OFF
 *    Touch on either- LED2 == ON
 *
 *  Control LED3 in the kit to show the slider status:
 *    No touch- LED3 == OFF
 *    Touch- LED3 == ON with brightness corresponding to touch position on slider
 *
 *
 *******************************************************************************/
void led_control()
{
//    cy_stc_capsense_touch_t *slider_touch_info;
//    uint16_t slider_pos;

    if(Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
    {
//        if(Cy_CapSense_IsWidgetActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context))
//        {
//            /* Get slider status */
//            slider_touch_info = Cy_CapSense_GetTouchInfo(
//                    CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context);
//            slider_pos = slider_touch_info->ptrPosition->x;
//
//            /* LED3 Turns ON and brightness changes when there is a touch detected on the slider */
//            Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, slider_pos*100);
//        }
        if(0u != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON0_WDGT_ID, &cy_capsense_context) || 0u != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_BUTTON1_WDGT_ID, &cy_capsense_context))
        {
            /* LED2 Turns ON when there is a touch detected on the CSD or CSX button*/
            Cy_GPIO_Write(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM, CYBSP_LED_ON);
            ACTIVEText(0x5u, "ON");
        }
    }
    else
    {
        /* Turn OFF LEDs */
        Cy_GPIO_Write(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM, CYBSP_LED_OFF);
        Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, 0);
        ACTIVEText(0x6u, "OFF");
    }
}
#endif

/*******************************************************************************
 * Function Name: register_callback
 ********************************************************************************
 *
 * Summary:
 *  Register Deep Sleep callbacks for EzI2C, SPI components
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void register_callback(void)
{
    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&deepSleepCb);
}

/*******************************************************************************
 * Function Name: deep_sleep_callback
 ********************************************************************************
 *
 * Summary:
 * Deep Sleep callback implementation. Waits for the completion of SPI transaction.
 * And change the SPI GPIOs to highZ while transition to deep-sleep and vice-versa
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 *******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(
        cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_CHECK_FAIL:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }
    return ret_val;
}

#if ACTIVE_SPIM_ENABLED
/*******************************************************************************
 * Function Name: mSPI_Interrupt
 *******************************************************************************
 *
 * Invokes the Cy_SCB_SPI_Interrupt() PDL driver function.
 *
 ******************************************************************************/
void mSPI_Interrupt(void)
{
    Cy_SCB_SPI_Interrupt(mSPI_HW, &mSPI_context);
}

/*******************************************************************************
 * Function Name: initMaster
 *******************************************************************************
 *
 * Summary:
 * This function initializes the SPI master based on the configuration done in
 * design.modus file.
 *
 * Parameters:
 * None
 *
 * Return:
 * uint32_t - Returns INIT_SUCCESS if the initialization is successful.
 * Otherwise it returns INIT_FAILURE
 *
 ******************************************************************************/
uint32_t initMaster(void)
{
    cy_en_scb_spi_status_t result;
    cy_en_sysint_status_t sysSpistatus;

    /* Configure the SPI block */

    result = Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context);
    if( result != CY_SCB_SPI_SUCCESS)
    {
        return(INIT_FAILURE);
    }

    /* Set active slave select to line 0 */
//    Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT0);

    /* Populate configuration structure */

    const cy_stc_sysint_t mSPI_SCB_IRQ_cfg =
    {
            .intrSrc      = mSPI_IRQ,
            .intrPriority = mSPI_INTR_PRIORITY
    };

    /* Hook interrupt service routine and enable interrupt */
    sysSpistatus = Cy_SysInt_Init(&mSPI_SCB_IRQ_cfg, &mSPI_Interrupt);
    if(sysSpistatus != CY_SYSINT_SUCCESS)
    {
        return(INIT_FAILURE);
    }
    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(mSPI_IRQ);

    /* Enable the SPI Master block */
    Cy_SCB_SPI_Enable(mSPI_HW);

    /* Initialization completed */
    return(INIT_SUCCESS);
}


/*******************************************************************************
 * Function Name: sendPacket
 *******************************************************************************
 *
 * Summary:
 * This function sends the data to the slave.
 *
 * Parameters:
 * txBuffer - Pointer to the transmit buffer
 * transferSize - Number of bytes to be transmitted
 *
 * Return:
 * cy_en_scb_spi_status_t - CY_SCB_SPI_SUCCESS if the transaction completes
 * successfully. Otherwise it returns the error status
 *
 ******************************************************************************/
cy_en_scb_spi_status_t sendPacket(uint8_t *txBuffer, uint32_t transferSize)
{
    cy_en_scb_spi_status_t masterStatus;

    /* Initiate SPI Master write transaction. */
    masterStatus = Cy_SCB_SPI_Transfer(mSPI_HW, txBuffer, NULL,
                                       transferSize, &mSPI_context);

    /* Blocking wait for transfer completion */
    while (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE &
                   Cy_SCB_SPI_GetTransferStatus(mSPI_HW, &mSPI_context)))
    {
    }

    return (masterStatus);
}
#endif

// ACTIVEPRO firmware tool stuff ->>>>

//************** MAKE YOUR CHANGES BELOW ONLY *************************
// These defines are an example using the Cypress PSoC5LP Microcontroller

#ifdef ACTIVE_DEBUG_ON

// Change #1 - Add any includes that you need for the below defines of your hardware interface to the ACTIVE bus
//#include "project.h"  // for PSoC Creator Projects

// CHANGE #2 - Select the mode of the ACTIVE bus to use
#define ACTIVE_ONE_WIRE_UART    // Uncomment this line to use a single wire (from a hardware UART) for the ACTIVE interface
//#define ACTIVE_TWO_WIRE_SPI     // Uncomment this line to use two wires (from a hardware SPI block - Clock and Data) for the ACTIVE interface
//#define ACTIVE_TWO_WIRE_GPIO      // Uncomment this line to use two wires (from standard GPIO - Clock and Data) for the ACTIVE interface

#ifdef ACTIVE_TWO_WIRE_SPI
// CHANGE #3 - FOR ACTIVE 2-wire SPI Mode:  Defines your routine name to send a single byte to the SPI block
// The SPI block must be configured and enabled elsewhere in your firmware.
    #define ACTIVESPITx(x)     SPIM_WriteTxData(x)
#endif

#ifdef ACTIVE_ONE_WIRE_UART
// CHANGE #6 - FOR one wire (UART) ACTIVE Mode:  Defines your routine name to send a single byte to the UART
// The UART must be configured and enabled elsewhere in your firmware.
//    #define ACTIVEUartTx(x)     DBG_PutChar(x)
    #define ACTIVEUartTx(x)     while(!Cy_SCB_UART_Put(CYBSP_UART_HW, x));
#endif

#ifdef ACTIVE_TWO_WIRE_GPIO
// CHANGE #3 - FOR ACTIVE 2-wire GPIO  Mode:  Defines that set the GPIO pins to High and Low levels and Toggles
// These should be as fast as possible, but not faster than 20.8ns (48MHz)
// These two GPIO are a Clock and a Data line.  They must be setup as outputs
// elsewhere in your firmware during system initialization.
#define ACTIVEClockLow       Cy_GPIO_Write(ACTIVE_SPI_CLK_PORT, ACTIVE_SPI_CLK_PIN, 0UL);
#define ACTIVEClockHigh      Cy_GPIO_Write(ACTIVE_SPI_CLK_PORT, ACTIVE_SPI_CLK_PIN, 1UL);
#define ACTIVEDataLow        Cy_GPIO_Write(ACTIVE_SPI_MOSI_PORT, ACTIVE_SPI_MOSI_PIN, 0UL);
#define ACTIVEDataHigh       Cy_GPIO_Write(ACTIVE_SPI_MOSI_PORT, ACTIVE_SPI_MOSI_PIN, 1UL);
#endif

#define MAX_ACTIVE_LENGTH 255 // This defines the maximum length of any debug text message

// CHANGE #7 - Type defines for your platform
// Copy these function prototypes to your header files to define the API
void ACTIVEValue( int channel, int value );             // Output a Value for this channel
void ACTIVEText( int channel, char *string );           // Output Text for this channel
void ACTIVEprintf( int channel, char *format, ... );    // printf-like function with variable argument list

//************** MAKE YOUR CHANGES ABOVE ONLY *************************

// Define the SendACTIVEByte Routine that sends bytes to the hardware interface
#ifdef ACTIVE_ONE_WIRE_UART   // This is a 1-wire UART interface to your processors UART block

void SendACTIVEByte( unsigned char value )
{
    ACTIVEUartTx( value );
}
#else
#ifdef ACTIVE_TWO_WIRE_SPI   // This is a 2-wire SPI interface to your processors SPI block

void SendACTIVEByte( unsigned char value )
{
    ACTIVESPITx( value );
}
#else
#ifdef ACTIVE_TWO_WIRE_GPIO   // This is a 2-wire interface using GPIO pins
void SendACTIVEByte( int value )
{
    if (value & 0x80) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x40) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x20) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x10) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x08) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x04) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x02) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
    if (value & 0x01) {ACTIVEDataHigh;} else {ACTIVEDataLow;} ACTIVEClockHigh; ACTIVEClockLow;
}
#endif
#endif
#endif

//===============================================================================================
// Define the basic Value and Text ACTIVE Message transmit routines
//===============================================================================================

//void ACTIVEValue( int channel, int value )
//{
////    ControlValueText_Write(5);
//	SendACTIVEByte( 0x7F );             // Every ACTIVE packet starts with a 0x7F
//	SendACTIVEByte( channel & 0x3F );	// Type and Channel: Bit 7=0, Bit 6=0 Means Value, Bits 5:0 means the channel (0-63)
//
//    while(value & 0xFFFFFFC0)           // Send out the bits of the value.
//    {
//        SendACTIVEByte( value & 0x3F ); // Bit 7=0, Bit 6=0 means not the last byte, Bits 5:0 are the next 6 LSBs of the value
//        value = value >> 6;
//    }
//   SendACTIVEByte( value | 0x40 );     // Bit 7=0, Bit 6=1 means the last byte, Bits 5:0 are the next 6 LSBs of the value
//}

#ifdef ACTIVE_TWO_WIRE_SPI
void ACTIVEValue( int channel, int value )
{
    char done = 0;
    char positive;

    /* Wait until TX FIFO has a place */

    while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
    CY_SET_REG8(SPIM_TXDATA_PTR, ( 0x7F ));             // Every ACTIVE packet starts with a 0x7F
    while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
    CY_SET_REG8(SPIM_TXDATA_PTR, ( channel & 0x3F ));   // Type and Channel: Bit 7=0, Bit 6=0 Means Value, Bits 5:0 means the channel (0-63)

    if (value >= 0)     // Positive values
        positive = 1;
    else
        positive = 0;

    while(!done)
    {
        if ((positive && (value >= 32)) || (!positive && (value < -32)))
        {
            while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
            CY_SET_REG8(SPIM_TXDATA_PTR, ( value & 0x3F )); // Bit 7=0, Bit 6=0 means not the last byte, Bits 5:0 are the next 6 LSBs of the value
            value = value >> 6;
        }
        else
        {
            while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
            CY_SET_REG8(SPIM_TXDATA_PTR, ( (value & 0x3F ) | 0x40 ));     // Bit 7=0, Bit 6=1 means the last byte, Bits 5:0 are the next 6 LSBs of the value
            done = 1;
        }
    }
}



void ACTIVEText( int channel, char *string )
{
    int bytes = 2;

//    ControlValueText_Write(10);

    /* Wait until TX FIFO has a place */

    while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
    CY_SET_REG8(SPIM_TXDATA_PTR, ( 0x7F ));                     // Every ACTIVE packet starts with a 0x7F
	while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
    CY_SET_REG8(SPIM_TXDATA_PTR, ( 0x40 | (channel & 0x3F) ));	// Type and Channel: Bit 7=0, Bit 6=1 Means Text, Bits 5:0 means the channel (0-63)

    while(*string)
    {
        if (bytes++ > MAX_ACTIVE_LENGTH)
            break;
        while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
        CY_SET_REG8(SPIM_TXDATA_PTR, ( *string & 0x7F ));     // Send the ascii characters
        string++;

    }
    while(0u == (SPIM_TX_STATUS_REG & SPIM_STS_TX_FIFO_NOT_FULL)){;}
    CY_SET_REG8(SPIM_TXDATA_PTR, ( 0 ));                        // Terminate the string
}
#else
void ACTIVEValue( int channel, int value )
{
    char done = 0;
    char positive;

	SendACTIVEByte( 0x7F );             // Every ACTIVE packet starts with a 0x7F
	SendACTIVEByte( channel & 0x3F );   // Type and Channel: Bit 7=0, Bit 6=0 Means Value, Bits 5:0 means the channel (0-63)

    if (value >= 0)     // Positive values
        positive = 1;
    else
        positive = 0;

    while(!done)
    {
        if ((positive && (value >= 32)) || (!positive && (value < -32)))
        {
            SendACTIVEByte( value & 0x3F ); // Bit 7=0, Bit 6=0 means not the last byte, Bits 5:0 are the next 6 LSBs of the value
            value = value >> 6;
        }
        else
        {
            SendACTIVEByte( (value & 0x3F ) | 0x40 );     // Bit 7=0, Bit 6=1 means the last byte, Bits 5:0 are the next 6 LSBs of the value
            done = 1;
        }
    }
}



void ACTIVEText( int channel, char *string )
{
    int bytes = 2;

    SendACTIVEByte( 0x7F );                     // Every ACTIVE packet starts with a 0x7F
	SendACTIVEByte( 0x40 | (channel & 0x3F) );	// Type and Channel: Bit 7=0, Bit 6=1 Means Text, Bits 5:0 means the channel (0-63)

    while(*string)
    {
        if (bytes++ > MAX_ACTIVE_LENGTH)
            break;
        SendACTIVEByte( *string & 0x7F );     // Send the ascii characters
        string++;

    }
    SendACTIVEByte( 0 );                        // Terminate the string
}
#endif

//===============================================================================================
// Define the printf-like ACTIVE message routine
//===============================================================================================

#include "stdio.h"
#include <stdarg.h>   // va_list, va_start, and va_end
char ACTIVEstr[MAX_ACTIVE_LENGTH];
void ACTIVEprintf( int channel, char *format, ... )
{
    va_list arglist;
    va_start( arglist, format );
    vsprintf( ACTIVEstr, format, arglist );
    va_end( arglist );
    ACTIVEText( channel, ACTIVEstr );
};

#else       // ACTIVE Debug is turned off, so make empty stubs for all routines
void ACTIVEText( int channel, char *string ) {};
void ACTIVEValue( int channel, int value ) {};
void ACTIVEprintf( int channel, char *format, ... ) {};
#endif
/* [] END OF FILE */
