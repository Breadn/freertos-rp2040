/**
 * Copyright (C) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

// #include "coines.h"  TODO: include coines sdk
#include "pico/stdlib.h"    // use pico-sdk utilities instead
#include "pico/binary_info.h"    // for picotool use
#include "bmp5_defs.h"
#include "bmp5_common.h"

/******************************************************************************/
/*!                         Macro definitions                                 */

/*! BMP5 shuttle id */
#define BMP5_SHUTTLE_ID_PRIM  UINT16_C(0x1B3)
#define BMP5_SHUTTLE_ID_SEC   UINT16_C(0x1D3)

/* I2C Speed */
#define I2C_SPEED_HZ (uint8_t)100000     // 100kHz - classic speed, 400kHz - fast speed, 3.4MHz - high speed

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t rslt;
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    // configure read at register addr
    rslt = i2c_write_blocking(i2c_default, device_addr, &reg_addr, 1, true);   // true to retain bus ctrl since another transaction will be made
    if (rslt == PICO_ERROR_GENERIC || rslt != 1) {
        return BMP5_E_COM_FAIL;
    }

    // read actual data
    rslt = i2c_read_blocking(i2c_default, device_addr, reg_data, length, false);
    if (rslt == PICO_ERROR_GENERIC || rslt != length) {
        return BMP5_E_COM_FAIL;
    }

    return BMP5_OK;
}

/*!
 * I2C write function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t rslt;
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    printf("IMPLEMENT THIS (Did i2c write!)\n");

    // TODO: Implement

    return BMP5_E_COM_FAIL;
}

/*!
 * SPI read function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // TODO: implement
    return BMP5_E_DEV_NOT_FOUND;
}

/*!
 * SPI write function map to COINES platform
 */
BMP5_INTF_RET_TYPE bmp5_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // TODO: implement
    return BMP5_E_DEV_NOT_FOUND;
}

/*!
 * Delay function map to COINES platform
 */
void bmp5_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    busy_wait_us_32(period);
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmp5_error_codes_print_result(const char api_name[], int8_t rslt)
{
    if (rslt != BMP5_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP5_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer\r\n", rslt);
        }
        else if (rslt == BMP5_E_COM_FAIL)
        {
            printf("Error [%d] : Communication failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_CHIP_ID)
        {
            printf("Error [%d] : Invalid chip id\r\n", rslt);
        }
        else if (rslt == BMP5_E_POWER_UP)
        {
            printf("Error [%d] : Power up error\r\n", rslt);
        }
        else if (rslt == BMP5_E_POR_SOFTRESET)
        {
            printf("Error [%d] : Power-on reset/softreset failure\r\n", rslt);
        }
        else if (rslt == BMP5_E_INVALID_POWERMODE)
        {
            printf("Error [%d] : Invalid powermode\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmp5_interface_init(struct bmp5_dev *bmp5_dev, uint8_t intf)
{
    int8_t rslt = BMP5_OK;

    if (bmp5_dev != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMP5_I2C_INTF)
        {
            printf("Note: Configuring for I2C Interface\n");

            dev_addr = BMP5_I2C_ADDR_PRIM;
            bmp5_dev->read = bmp5_i2c_read;
            bmp5_dev->write = bmp5_i2c_write;
            bmp5_dev->intf = BMP5_I2C_INTF;

            /* Holds the I2C device addr */
            bmp5_dev->intf_ptr = &dev_addr;

            /* Configure delay in microseconds */
            bmp5_dev->delay_us = bmp5_delay_us;

            /* i2c bus init */
            ace_init_i2c_bus();
        }
        /* Bus configuration : SPI */
        else if (intf == BMP5_SPI_INTF)
        {
            printf("SPI Interface is not supported on the ACE 3.0!\n");
            rslt = BMP5_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMP5_E_NULL_PTR;
    }

    return rslt;
}

// TODO: move into common ACE interface lib
void ace_init_i2c_bus() {
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning No I2C pins found
    puts("Default I2C pins were not defined");
#endif

    i2c_init(i2c_default, I2C_SPEED_HZ);
    
    // set sda, scl pins to i2c operation mode
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);

    // breadn-note: so we do already have internal pullups for the i2c bus...?
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

void bmp5_coines_deinit(void)
{
    i2c_deinit(i2c_default);
}
