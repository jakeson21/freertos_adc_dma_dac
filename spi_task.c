/*
 * spi_task.c
 *
 *  Created on: Oct 1, 2023
 *      Author: Fuguru
 */

#include <spi_task.h>
#include <driverlib/sysctl.h>

uint32_t SPI_xfer(uint16_t inTxData, int8_t inFssHold)
{
    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    uint32_t dummy;
    while(MAP_SSIDataGetNonBlocking(SSI0_BASE, &dummy))
    {
    }

    // Assert EN low
    if ((SPI_HOLD_ACTIVE == inFssHold) || (SPI_HOLD_CLR == inFssHold))
    {
        MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);
    }

    //
    // Send inLength bytes of data.
    //
    // Wait for Tx buffer to clear
    while(MAP_SSIBusy(SSI0_BASE)) {}

    //
    // Send the data using the "blocking" put function.  This function
    // will wait until there is room in the send FIFO before returning.
    // This allows you to assure that all the data you send makes it into
    // the send FIFO.
    //
    MAP_SSIDataPut(SSI0_BASE, inTxData);

    // De-assert EN on last byte
    if ((inFssHold == SPI_HOLD_CLR))
    {
        while(MAP_SSIBusy(SSI0_BASE)) {}
        MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
    }

    return 0;
}
