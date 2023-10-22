/*
 * spi_task.h
 *
 *  Created on: Oct 1, 2023
 *      Author: Fuguru
 */

#ifndef SPI_TASK_H_
#define SPI_TASK_H_

#include <stdbool.h>
#include <stdint.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_ints.h>
#include <inc/hw_nvic.h>
#include <inc/hw_gpio.h>

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/ssi.h>

typedef enum {
    SPI_HOLD_NONE = 0,
    SPI_HOLD_ACTIVE,
    SPI_HOLD_CLR
} spi_cs_hold_e;

uint32_t SPI_xfer(uint16_t inTxData, int8_t inFssHold);


#endif /* SPI_TASK_H_ */
