//*****************************************************************************
//
// freertos_demo.c - Simple FreeRTOS example.
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

// https://www.freertos.org/fr-content-src/uploads/2018/07/161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf
// https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/367620/ssi-udma-full-example-tm4c1294xl
// https://github.com/vuquangtrong/tiva-c/blob/master/6_Audio/AudioOutput/AudioOutput.cpp

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_adc.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_udma.h"
#include "inc/hw_uart.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include <driverlib/ssi.h>
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "led_task.h"
#include "switch_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include "drivers/rgb.h"

#include <spi_task.h>


//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>FreeRTOS Example (freertos_demo)</h1>
//!
//! This application demonstrates the use of FreeRTOS on Launchpad.
//!
//! The application blinks the user-selected LED at a user-selected frequency.
//! To select the LED press the left button and to select the frequency press
//! the right button.  The UART outputs the application status at 115,200 baud,
//! 8-n-1 mode.
//!
//! This application utilizes FreeRTOS to perform the tasks in a concurrent
//! fashion.  The following tasks are created:
//!
//! - An LED task, which blinks the user-selected on-board LED at a
//!   user-selected rate (changed via the buttons).
//!
//! - A Switch task, which monitors the buttons pressed and passes the
//!   information to LED task.
//!
//! In addition to the tasks, this application also uses the following FreeRTOS
//! resources:
//!
//! - A Queue to enable information transfer between tasks.
//!
//! - A Semaphore to guard the resource, UART, from access by multiple tasks at
//!   the same time.
//!
//! - A non-blocking FreeRTOS Delay to put the tasks in blocked state when they
//!   have nothing to do.
//!
//! For additional details on FreeRTOS, refer to the FreeRTOS web page at:
//! http://www.freertos.org/
//
//*****************************************************************************

// 2.03 PE0 AIN3 (maybe damaged?)
// 1.05 PE4 AIN9 ?
// AIN0 PE3

void MyTimerCallback( TimerHandle_t xTimer );
uint32_t ulCallCount = 0;
//uint32_t ui32ADCValues[8] = {0,0,0,0,0,0,0,0};
float fADCValue = 0.0;
#define SSN 0
//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(TaskHandle_t xTask,
                              char * pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    MAP_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Definition for ADC buffer size.
//
//*****************************************************************************
// Ensure SysCtlClockGet() is evenly divisible by ADC_SAMPLE_RATE_HZ
// Ensure ADC_SAMPLE_RATE_HZ is evenly divisible by ADC_SAMPLE_BUF_SIZE
#define ADC_SAMPLE_BUF_SIZE     400
#define ADC_SAMPLE_RATE_HZ      20000

#define SSI_SAMPLE_BUF_SIZE     ADC_SAMPLE_BUF_SIZE

//*****************************************************************************
//
// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif

//*****************************************************************************
//
// Global buffers to store ADC sample data.
//
//*****************************************************************************
#pragma DATA_ALIGN(pui16ADCBuffer1, 128)
#pragma DATA_ALIGN(pui16ADCBuffer2, 128)
static uint16_t pui16ADCBuffer1[ADC_SAMPLE_BUF_SIZE];
static uint16_t pui16ADCBuffer2[ADC_SAMPLE_BUF_SIZE];

static uint16_t pui16Buffer1[ADC_SAMPLE_BUF_SIZE];
static uint16_t pui16Buffer2[ADC_SAMPLE_BUF_SIZE];

static uint16_t pui16SSIBuffer1[SSI_SAMPLE_BUF_SIZE];
static uint16_t pui16SSIBuffer2[SSI_SAMPLE_BUF_SIZE];

//static uint16_t puiDebugSsiVal = 0;
uint32_t ui32Count, ui32AverageResult1, ui32AverageResult2;
uint32_t ui32SamplesTaken = 0;

uint16_t ui16TimeBase[2] = {0, 0};
uint8_t g_ui8PingPong = 0;
uint8_t g_ui8DacStatePrev = 0;
uint32_t g_ui32DacOutputCount = 0;

//*****************************************************************************
//
// The count of SSI0 buffers filled
//
//*****************************************************************************
static uint32_t g_ui32Count = 0;
static uint32_t g_ui32SSITxFFCount = 0;
static uint32_t g_ui32SSIPriModeStopCount = 0;
static uint32_t g_ui32SSIAltModeStopCount = 0;


//*****************************************************************************
//
// Each possible state of the fill status for an ADC buffer.
//
//*****************************************************************************
enum BUFFER_STATUS
{
    EMPTY,
    EMPTYING,
    FILLING,
    FULL
};

//*****************************************************************************
//
// Global variable to keep track of the fill status of each ADC buffer.
//
//*****************************************************************************
static enum BUFFER_STATUS pui32BufferStatus[2];
static enum BUFFER_STATUS pui32SSIBufferStatus[2];

//*****************************************************************************
//
// The count of uDMA errors.  This value is incremented by the uDMA error
// handler.
//
//*****************************************************************************
static uint32_t g_ui32DMAErrCount = 0u;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The interrupt handler for uDMA errors.  This interrupt will occur if the
// uDMA encounters a bus error while trying to perform a transfer.  This
// handler just increments a counter if an error occurs.
//
//*****************************************************************************
void
uDMAErrorHandler(void)
{
    uint32_t ui32Status;

    //
    // Check for uDMA error bit.
    //
    ui32Status = MAP_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        MAP_uDMAErrorStatusClear();
        g_ui32DMAErrCount++;
    }
}


//*****************************************************************************
//
// SSI0 Interrupt Handler
//
//*****************************************************************************
void
SSI0IntHandler(void)
{
//    uint32_t ui32Status;
    uint32_t ui32Mode;

//    ui32Status = MAP_SSIIntStatus(SSI0_BASE, 1);
    MAP_SSIIntClear(SSI0_BASE, SSI_RXTO | SSI_RXOR);

    ui32Mode = MAP_uDMAChannelModeGet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT);
    if (ui32Mode == UDMA_MODE_STOP && (pui32SSIBufferStatus[0] == EMPTYING))
    {
        g_ui32SSIPriModeStopCount += SSI_SAMPLE_BUF_SIZE;
        pui32SSIBufferStatus[0] = EMPTY;
    }

    ui32Mode = MAP_uDMAChannelModeGet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT);
    if (ui32Mode == UDMA_MODE_STOP && (pui32SSIBufferStatus[1] == EMPTYING))
    {
        g_ui32SSIAltModeStopCount += SSI_SAMPLE_BUF_SIZE;
        pui32SSIBufferStatus[1] = EMPTY;
    }
}

void ConfigureUdmaAndSSI()
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS); // don't manually toggle this
    MAP_GPIOPinConfigure(GPIO_PA4_SSI0RX);
    MAP_GPIOPinConfigure(GPIO_PA5_SSI0TX);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx
    //      PA4 - SSI0Rx
    //      PA3 - SSI0Fss
    //      PA2 - SSI0CLK
    // TODO: change this to select the port/pin you are using.
    //
    MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    // Override default 2mA setting, set to 8mA per design guide
//    MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 |GPIO_PIN_3 | GPIO_PIN_2, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);

    // clock supply, master mode, 1MHz SSI frequency, and 8-bit data.
    //
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    MAP_SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);
#else
    MAP_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 2000000, 12); // run this a little faster than the ADC so it doesn't fall behind
    // for 10k => 134500
#endif

    MAP_SysCtlDelay(10);

    //
    // Enable the SSI0 module.
    //
    MAP_SSIEnable(SSI0_BASE);

    //
    // Enable the uDMA interface for TX channel
    //
    MAP_SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);

    //
    // Put the attributes in a known state for the uDMA SSI0TX channel.  These
    // should already be disabled by default.
    //
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);


    //
    // Set the USEBURST attribute for the uDMA SSI0 channel.  This will force
    // the controller to always use a burst when transferring data from the
    // TX buffer to the SSI0.  This is somewhat more efficient bus usage than
    // the default which allows single or burst transfers.
    //
    MAP_uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);

    //
    // Configure the control parameters for the primary control structure for
    // SSI0TX.  The primary control structure is used for the "A"
    // part of the ping-pong receive.  The transfer data size is 16 bits, the
    // source address does not increment since it will be reading from a
    // register.  The destination address increment is 16-bits.  The
    // arbitration size is set to one byte transfers.
    //
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_1);

    //
    // Configure the control parameters for the alternate control structure for
    // SSI0TX.  The alternate control structure is used for the
    // "B" part of the ping-pong receive.  The configuration is identical to
    // the primary/A control structure.
    //
//    MAP_uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT, UDMA_SIZE_16 |
//                          UDMA_SRC_INC_16 | UDMA_DST_INC_NONE | UDMA_ARB_8);

    //
    // Set up the transfer parameters for the SSI0 primary control structure
    // The mode is set to ping-pong, the transfer source is pui16SSIBuffer1
    // Sequence Result FIFO 0 register, and the destination is the receive
    // "A" buffer.  The transfer size is set to match the size of the buffer.
    //
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC,
                               &pui16SSIBuffer1,
                               (void *)(SSI0_BASE + SSI_O_DR),
                               SSI_SAMPLE_BUF_SIZE);

    //
    // Set up the transfer parameters for the SSI0 primary control structure
    // The mode is set to ping-pong, the transfer source is pui16SSIBuffer2
    // Sequence Result FIFO 0 register, and the destination is the receive
    // "B" buffer.  The transfer size is set to match the size of the buffer.
    //
//    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT,
//                           UDMA_MODE_PINGPONG,
//                           &pui16SSIBuffer2,
//                           (void *)(SSI0_BASE + SSI_O_DR),
//                           SSI_SAMPLE_BUF_SIZE);


    //
    // Enables DMA channel so it can perform transfers.  As soon as the
    // channels are enabled, the peripheral will issue a transfer request and
    // the data transfers will begin.
    //
    MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);


//    MAP_SSIIntEnable(SSI0_BASE, SSI_TXFF);
//    SSIIntRegister(SSI0_BASE, SSI0IntHandler);
//    SSIIntEnable(SSI0_BASE, SSI_DMATX);

    //
    // Enable the interrupt for SSI0 on the processor (NVIC).
    //
    MAP_IntEnable(INT_SSI0);

}

//*****************************************************************************
//
// Interrupt handler for ADC0 Sequence Zero.
//
//*****************************************************************************
void
ADCSeq0Handler(void)
{
    //
    // Clear the Interrupt Flag.
    //
    MAP_ADCIntClear(ADC0_BASE, 0);

    //
    // Determine which buffer as been filled based on the UDMA_MODE_STOP flag
    // and update the buffer status.
    //
    if ((MAP_uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT) ==
                            UDMA_MODE_STOP) &&
                           (pui32BufferStatus[0] == FILLING))
    {
        pui32BufferStatus[0] = FULL;
        pui32BufferStatus[1] = FILLING;

        MAP_GPIOPinWrite(RED_GPIO_BASE, RED_GPIO_PIN, RED_GPIO_PIN);
    }
    if ((MAP_uDMAChannelModeGet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT) ==
                                 UDMA_MODE_STOP) &&
                                (pui32BufferStatus[1] == FILLING))
    {
        pui32BufferStatus[0] = FILLING;
        pui32BufferStatus[1] = FULL;

        MAP_GPIOPinWrite(RED_GPIO_BASE, RED_GPIO_PIN, 0);
    }
}

void ConfigureUdmaAndAdc()
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    //
    // Wait for the ADC0 module to be ready.
    //
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }

    //
    // Put the attributes in a known state for the uDMA ADC0 channel.  These
    // should already be disabled by default.
    //
    MAP_uDMAChannelAttributeDisable(UDMA_CHANNEL_ADC0,
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_HIGH_PRIORITY |
                                UDMA_ATTR_REQMASK);

    //
    // Configure the control parameters for the primary control structure for
    // the ADC0 channel.  The primary control structure is used for the "A"
    // part of the ping-pong receive.  The transfer data size is 16 bits, the
    // source address does not increment since it will be reading from a
    // register.  The destination address increment is 16-bits.  The
    // arbitration size is set to one byte transfers.
    //
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    //
    // Configure the control parameters for the alternate control structure for
    // the ADC0 channel.  The alternate control structure is used for the
    // "B" part of the ping-pong receive.  The configuration is identical to
    // the primary/A control structure.
    //
    MAP_uDMAChannelControlSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT, UDMA_SIZE_16 |
                          UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_1);

    //
    // Set up the transfer parameters for the ADC0 primary control structure
    // The mode is set to ping-pong, the transfer source is the ADC Sample
    // Sequence Result FIFO 0 register, and the destination is the receive
    // "A" buffer.  The transfer size is set to match the size of the buffer.
    //
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                           UDMA_MODE_PINGPONG,
                           (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                           &pui16ADCBuffer1, ADC_SAMPLE_BUF_SIZE);

    //
    // Set up the transfer parameters for the ADC0 primary control structure
    // The mode is set to ping-pong, the transfer source is the ADC Sample
    // Sequence Result FIFO 0 register, and the destination is the receive
    // "B" buffer.  The transfer size is set to match the size of the buffer.
    //
    MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                           UDMA_MODE_PINGPONG,
                           (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                           &pui16ADCBuffer2, ADC_SAMPLE_BUF_SIZE);

    //
    // Set the USEBURST attribute for the uDMA ADC0 channel.  This will force
    // the controller to always use a burst when transferring data from the
    // TX buffer to the UART.  This is somewhat more efficient bus usage than
    // the default which allows single or burst transfers.
    //
    MAP_uDMAChannelAttributeEnable(UDMA_CHANNEL_ADC0, UDMA_ATTR_USEBURST);

    //
    // Enables DMA channel so it can perform transfers.  As soon as the
    // channels are enabled, the peripheral will issue a transfer request and
    // the data transfers will begin.
    //
    MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC0);

    //
    // Use ADC0 sequence 0 to sample channel 3 once for each timer period.
    //
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PIOSC | ADC_CLOCK_RATE_HALF, 1);

    //
    // Wait for the clock configuration to set.
    //
    MAP_SysCtlDelay(10);

    //
    // Disable the ADC0 sequence 0 interrupt on the processor (NVIC).
    //
    MAP_IntDisable(INT_ADC0SS0);

    //
    // Disable interrupts for ADC0 sample sequence 0 to configure it.
    //
    MAP_ADCIntDisable(ADC0_BASE, 0);

    //
    // Disable ADC0 sample sequence 0.  With the sequence disabled, it is now
    // safe to load the new configuration parameters.
    //
    MAP_ADCSequenceDisable(ADC0_BASE, 0);

    //
    // Enable sample sequence 0 with a processor signal trigger.  Sequence 0
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    //
    MAP_ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);

    //
    // Configure step 0 on sequence 0.  Sample channel 3 (ADC_CTL_CH3) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 0 (ADC_CTL_END).  Sequence
    // 0 has 8 programmable steps.  Since we are only doing a single conversion
    // using sequence 0 we will only configure step 0.  For more information
    // on the ADC sequences and steps, reference the datasheet.
    //
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH1 | ADC_CTL_END |
                             ADC_CTL_IE);

    //
    // Since sample sequence 0 is now configured, it must be enabled.
    //
    MAP_ADCSequenceEnable(ADC0_BASE, 0);

    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    MAP_ADCIntClear(ADC0_BASE, 0);

    //
    // Enables the DMA channel for the ADC0 sample sequence 0.
    //
    MAP_ADCSequenceDMAEnable(ADC0_BASE, 0);

    //
    // Enable the ADC 0 sample sequence 0 interrupt.
    //
    MAP_ADCIntEnable(ADC0_BASE, 0);

    //
    // Enable the interrupt for ADC0 sequence 0 on the processor (NVIC).
    //
    MAP_IntEnable(INT_ADC0SS0);

    //
    // Configure a 16-bit periodic timer. TIMER_CFG_SPLIT_PAIR
    //
    MAP_TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC | TIMER_CFG_A_PERIODIC);

    //
    // Set ADC sampling frequency to be 16KHz i.e. every 62.5uS.
    //
    MAP_TimerLoadSet(TIMER2_BASE, TIMER_A, (MAP_SysCtlClockGet()/ADC_SAMPLE_RATE_HZ) - 1);

    //
    // Enable the ADC trigger output for Timer A.
    //
    MAP_TimerControlTrigger(TIMER2_BASE, TIMER_A, true);

}

void HandleAdc()
{
    //
    // Check if the first buffer is full, if so process data.
    //
    if(pui32BufferStatus[0] == FULL)
    {
        //
        // Process the data in pui16ADCBuffer1 and clear buffer entries.
        //
        for(ui32Count = 0; ui32Count < ADC_SAMPLE_BUF_SIZE; ui32Count++)
        {
            ui32AverageResult1 += pui16ADCBuffer1[ui32Count];
            pui16Buffer1[ui32Count] = pui16ADCBuffer1[ui32Count];
        }

        //
        // Indicate the Buffer data as been processed so new data can
        // be stored.
        //
        pui32BufferStatus[0] = FILLING;
        pui32SSIBufferStatus[0] = FULL;

        //
        // Enable for another uDMA block transfer.
        //
        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                               &pui16ADCBuffer1, ADC_SAMPLE_BUF_SIZE);
        //
        // Enable DMA channel so it can perform transfers.
        //
        MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC0);


        //
        // Track the number of samples taken and update the average.
        //
        ui32SamplesTaken += ADC_SAMPLE_BUF_SIZE;
    }

    //
    // Check if the second buffer is full, if so process data.
    //
    if(pui32BufferStatus[1] == FULL)
    {
        //
        // Process the data in pui16ADCBuffer2 and clear buffer entries.
        //
        for(ui32Count = 0; ui32Count < ADC_SAMPLE_BUF_SIZE; ui32Count++)
        {
            ui32AverageResult2 += pui16ADCBuffer2[ui32Count];
            pui16Buffer2[ui32Count] = pui16ADCBuffer2[ui32Count];
        }

        //
        // Indicate the Buffer data as been processed so new data can
        // be stored.
        //
        pui32BufferStatus[1] = FILLING;
        pui32SSIBufferStatus[1] = FULL;

        //
        // Enable for another uDMA block transfer.
        //
        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_ADC0 | UDMA_ALT_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(ADC0_BASE + ADC_O_SSFIFO0),
                               &pui16ADCBuffer2, ADC_SAMPLE_BUF_SIZE);

        //
        // Enable DMA channel so it can perform transfers.
        //
        MAP_uDMAChannelEnable(UDMA_CHANNEL_ADC0);

        //
        // Track the number of samples taken and update the average.
        //
        ui32SamplesTaken += ADC_SAMPLE_BUF_SIZE;
    }
}

void
HandleDAC()
{
    uint32_t ui32Count;

    if(pui32SSIBufferStatus[0] == FULL)
    {
        g_ui32SSITxFFCount += SSI_SAMPLE_BUF_SIZE;

        for(ui32Count = 0; ui32Count < SSI_SAMPLE_BUF_SIZE; ui32Count++)
        {
            pui16SSIBuffer1[ui32Count] = pui16Buffer2[ui32Count];
        }

        pui32SSIBufferStatus[0] = EMPTYING;

//        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   pui16SSIBuffer1,
//                                   (void *)(SSI0_BASE + SSI_O_DR),
//                                   SSI_SAMPLE_BUF_SIZE);
//
//        MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

//        MAP_GPIOPinWrite(BLUE_GPIO_BASE, GREEN_GPIO_PIN, GREEN_GPIO_PIN);
        g_ui8PingPong ^= 1;
    }

    if(pui32SSIBufferStatus[1] == FULL)
    {
        g_ui32SSITxFFCount += SSI_SAMPLE_BUF_SIZE;

        for(ui32Count = 0; ui32Count < SSI_SAMPLE_BUF_SIZE; ui32Count++)
        {
            pui16SSIBuffer2[ui32Count] = pui16Buffer1[ui32Count];
        }

        pui32SSIBufferStatus[1] = EMPTYING;

//        MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_ALT_SELECT,
//                                   UDMA_MODE_PINGPONG,
//                                   pui16SSIBuffer2,
//                                   (void *)(SSI0_BASE + SSI_O_DR),
//                                   SSI_SAMPLE_BUF_SIZE);
//
//        MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

//        MAP_GPIOPinWrite(BLUE_GPIO_BASE, GREEN_GPIO_PIN, 0);
        g_ui8PingPong ^= 1;
    }
}

void Timer3AIntHandler(void)
{
    ROM_TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);

    if (g_ui8DacStatePrev != g_ui8PingPong)
    {
        MAP_GPIOPinWrite(BLUE_GPIO_BASE, GREEN_GPIO_PIN, GREEN_GPIO_PIN);
        g_ui32DacOutputCount = 0;
    }

    if (g_ui32DacOutputCount < SSI_SAMPLE_BUF_SIZE)
    {
        // Send DAC data
        if (g_ui8PingPong)
        {
            MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC,
                                       pui16SSIBuffer1 + g_ui32DacOutputCount,
                                       (void *)(SSI0_BASE + SSI_O_DR),
                                       1);

        }
        else
        {
            MAP_uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                                       UDMA_MODE_BASIC,
                                       pui16SSIBuffer2 + g_ui32DacOutputCount,
                                       (void *)(SSI0_BASE + SSI_O_DR),
                                       1);

        }
        MAP_uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);

        // Increment buffer position
        g_ui32DacOutputCount++;
    }
    else
    {
        // All data has been written
        MAP_GPIOPinWrite(BLUE_GPIO_BASE, GREEN_GPIO_PIN, 0);
        g_ui32Count++;
    }

    g_ui8DacStatePrev = g_ui8PingPong;
}

void
DoWork(void)
{
    if (ui16TimeBase[0])
    {
        uint16_t* pDataBuff;
        if (g_ui8PingPong)
        {
            pDataBuff = pui16Buffer1;
        }
        else
        {
            pDataBuff = pui16Buffer2;
        }

//        uint32_t k;
//        if (g_ui8PingPong)
//        {
//            for (k = 0; k<SSI_SAMPLE_BUF_SIZE; k++)
//            {
//                *pDataBuff = 20*k+2;
//                ++pDataBuff;
//            }
//        }
//        else
//        {
//            for (k = SSI_SAMPLE_BUF_SIZE; k>0; k--)
//            {
//                *pDataBuff = 20*k+2;
//                ++pDataBuff;
//            }
//        }

//        MAP_SysCtlDelay(10000);
    }

    // Update Timebase
    ui16TimeBase[0]++;
    if (ui16TimeBase[0]>=2)
        ui16TimeBase[0] = 0;
}


void
InitLED(void)
{
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the clocking to run at 50 MHz from the PLL.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    //
    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    //
    ConfigureUART();


    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);

    //
    // Enable the uDMA controller.
    //
    MAP_uDMAEnable();

    //
    // Enable the uDMA controller error interrupt.  This interrupt will occur
    // if there is a bus error during a transfer.
    //
    MAP_IntEnable(INT_UDMAERR);

    //
    // Point at the control table to use for channel control structures.
    //
    MAP_uDMAControlBaseSet(pui8ControlTable);

    //
    // Initialize the buffer status.
    //
    pui32BufferStatus[0] = FILLING;
    pui32BufferStatus[1] = EMPTY;

    ConfigureUdmaAndAdc();

    pui32SSIBufferStatus[0] = EMPTY;
    pui32SSIBufferStatus[1] = EMPTY;

    ConfigureUdmaAndSSI();

    int k;
    for (k = 0; k<SSI_SAMPLE_BUF_SIZE; k++)
    {
        pui16SSIBuffer1[k] = 0;
        pui16SSIBuffer2[k] = 0;
        pui16Buffer1[k] = 0;
        pui16Buffer2[k] = 0;
    }

    InitLED();

    //
    // Trigger the sample sequence.
    //
    ADCProcessorTrigger(ADC0_BASE, SSN);


    //
    // Print demo introduction.
    //
    UARTprintf("\n\nWelcome to the EK-TM4C123GXL FreeRTOS Demo!\n");


    // Setup Timer 3 for DAC SPI
    //
    // Enable the Timer0 peripheral
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    //
    // Wait for the Timer0 module to be ready.
    //
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3))
    {
    }

    //
    // Configure TimerA as a half-width one-shot timer, and TimerB as a
    // half-width edge capture counter.
    //
    MAP_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC | TIMER_CFG_A_PERIODIC);

    //
    // Set the count time for the the one-shot timer (TimerA).
    //
    MAP_TimerLoadSet(TIMER3_BASE, TIMER_A, (MAP_SysCtlClockGet()/ADC_SAMPLE_RATE_HZ) - 1);

    //
    // Configure the counter (TimerB) to count both edges.
    //
    MAP_TimerControlEvent(TIMER3_BASE, TIMER_A, TIMER_EVENT_BOTH_EDGES);

    //
    // Setup the interrupts for the timer timeouts.
    //
    MAP_IntEnable(INT_TIMER3A);
    MAP_TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);


    /* pdMS_TO_TICKS() takes a time in milliseconds as its only parameter, and evaluates
    to the equivalent time in tick periods. This example shows xTimeInTicks being set to
    the number of tick periods that are equivalent to 200 milliseconds. */
    // TickType_t xTimeInTicks = pdMS_TO_TICKS( 200 );

    //
    // Create a mutex to guard the UART.
    //
    g_pUARTSemaphore = xSemaphoreCreateMutex();

    //
    // Create the switch task.
    //
    if(SwitchTaskInit() != 0)
    {
        while(1) { }
    }

    TimerHandle_t xTimer = xTimerCreate((const portCHAR *)"MyTimer",
                                pdMS_TO_TICKS((1000 * ADC_SAMPLE_BUF_SIZE) / ADC_SAMPLE_RATE_HZ / 2),
                                pdTRUE,
                                NULL,
                                MyTimerCallback);
    if(xTimerStart(xTimer, 0) != pdPASS)
    {
        while(1) { }
    }


    //
    // Enable processor interrupts.
    //
    MAP_IntMasterEnable();

    //
    // Enable DAC timer
    //
    MAP_TimerEnable(TIMER3_BASE, TIMER_A);

    //
    // Enable Timer 0 which will start the whole application process.
    //
    MAP_TimerEnable(TIMER2_BASE, TIMER_A);

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //
    while(1)
    {
    }
}


void MyTimerCallback( TimerHandle_t xTimer )
{
    MAP_GPIOPinWrite(BLUE_GPIO_BASE, BLUE_GPIO_PIN, BLUE_GPIO_PIN);

    HandleAdc();

    HandleDAC();

    DoWork();

    MAP_GPIOPinWrite(BLUE_GPIO_BASE, BLUE_GPIO_PIN, 0);
}

