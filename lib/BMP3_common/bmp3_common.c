/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/** breadn-note: this file requires the coines sdk
 *      to function. its functionality has been
 *      temporarily supplemented via pico-sdk MCU
 *      functions
 * 
 * also useful: the i2c default bus identifier is from the pico-sdk,
 *      they are of the type 'i2c_inst_t *' from the define in
 *      'i2c.h' (#define i2c0 (&i2c0_inst) for HW block 0),
 *          which itself, 'i2c0_inst', is an extern variable of type 
 *          i2c_inst_t
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmp3.h"
// #include "coines.h"  // TODO: build/include bosch coines sdk
#include "pico/stdlib.h"    // use pico-sdk utilities instead
#include "pico/binary_info.h"    // for picotool use
#include "bmp3_common.h"

/*! BMP3 I2C Speed (Hz) */
#define I2C_SPEED_HZ (uint8_t)100000     // 100kHz - classic speed, 400kHz - fast speed, 3.4MHz - high speed

/* Variable to store the device address */
static uint8_t dev_addr;

/*!
 * I2C read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* breadn-note: replaced with custom functionality */
    // uint8_t device_addr = *(uint8_t*)intf_ptr;

    // (void)intf_ptr;

    // return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);

    uint8_t rslt;
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    // configure read at register addr
    rslt = i2c_write_blocking(i2c_default, device_addr, &reg_addr, 1, true);   // true to retain bus ctrl since another transaction will be made
    if (rslt == PICO_ERROR_GENERIC) {
        return BMP3_E_COMM_FAIL;
    }

    // read actual data
    rslt = i2c_read_blocking(i2c_default, device_addr, reg_data, len, false);
    if (rslt == PICO_ERROR_GENERIC) {
        return BMP3_E_COMM_FAIL;
    }

    return BMP3_OK;
}

/*!
 * I2C write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* breadn-note: replaced with custom functionality */
    // uint8_t device_addr = *(uint8_t*)intf_ptr;

    // (void)intf_ptr;

    // return coines_write_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);

    uint8_t rslt;
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    // configure write at register addr
    rslt = i2c_write_blocking(i2c_default, device_addr, &reg_addr, 1, true);
    if (rslt == PICO_ERROR_GENERIC) {
        return BMP3_E_COMM_FAIL;
    }

    rslt = i2c_write_blocking(i2c_default, device_addr, reg_data, len, false);
    if (rslt == PICO_ERROR_GENERIC) {
        return BMP3_E_COMM_FAIL;
    }

    return rslt;
}

/*!
 * SPI read function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* breadn-note: replaced with custom functionality */
    // uint8_t device_addr = *(uint8_t*)intf_ptr;

    // (void)intf_ptr;

    // return coines_read_spi(COINES_SPI_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
    

    int8_t rslt = BMP3_E_DEV_NOT_FOUND;

    printf("BMP3 spi read not implmented! (yet!)\n");

    return rslt;
}

/*!
 * SPI write function map to COINES platform
 */
BMP3_INTF_RET_TYPE bmp3_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    /* breadn-note: replaced with custom functionality */
    // uint8_t device_addr = *(uint8_t*)intf_ptr;

    // (void)intf_ptr;

    // return coines_write_spi(COINES_SPI_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
    
    
    int8_t rslt = BMP3_E_DEV_NOT_FOUND;

    printf("BMP3 spi write not implmented! (yet!)\n");

    return rslt;
}

/*!
 * Delay function map to COINES platform
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    /* breadn-note: replaced with custom functionality */
    // (void)intf_ptr;

    // coines_delay_usec(period);

    sleep_us(period);

}

void bmp3_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMP3_OK:

            /* Do nothing */
            break;
        case BMP3_E_NULL_PTR:
            printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMP3_E_COMM_FAIL:
            printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
            break;
        case BMP3_E_INVALID_LEN:
            printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            break;
        case BMP3_E_DEV_NOT_FOUND:
            printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMP3_E_CONFIGURATION_ERR:
            printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
            break;
        case BMP3_W_SENSOR_NOT_ENABLED:
            printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
            break;
        case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
            printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
            break;
        default:
            printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

/* breadn-note: important function that ties bmp3 device struct function pointer interfaces to this common.c IO */
BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf)
{
    /* breadn-note: replaced with custom functionality */
//     int8_t rslt = BMP3_OK;
//     struct coines_board_info board_info;
//
//     if (bmp3 != NULL)
//     {
//         int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
//         if (result < COINES_SUCCESS)
//         {
//             printf(
//                 "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
//                 " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
//             exit(result);
//         }
//
//         result = coines_get_board_info(&board_info);
//
// #if defined(PC)
//         setbuf(stdout, NULL);
// #endif
//
//         if (result == COINES_SUCCESS)
//         {
//             if ((board_info.shuttle_id != BMP3_SHUTTLE_ID))
//             {
//                 printf("! Warning invalid sensor shuttle \n ," "This application will not support this sensor \n");
//             }
//         }
//
//         (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
//         coines_delay_msec(1000);
//
//         /* Bus configuration : I2C */
//         if (intf == BMP3_I2C_INTF)
//         {
//             /* breadn-note: Bosch example of connecting I2C IO interface to bmp3 device */
//             printf("I2C Interface\n");
//             dev_addr = BMP3_ADDR_I2C_PRIM;
//             bmp3->read = bmp3_i2c_read;
//             bmp3->write = bmp3_i2c_write;
//             bmp3->intf = BMP3_I2C_INTF;
//
//             /* SDO pin is made low */
//             (void)coines_set_pin_config(COINES_SHUTTLE_PIN_SDO, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
//             (void)coines_config_i2c_bus(COINES_I2C_BUS_0, COINES_I2C_STANDARD_MODE);
//         }
//         /* Bus configuration : SPI */
//         else if (intf == BMP3_SPI_INTF)
//         {
//             printf("SPI Interface\n");
//             dev_addr = COINES_SHUTTLE_PIN_7;
//             bmp3->read = bmp3_spi_read;
//             bmp3->write = bmp3_spi_write;
//             bmp3->intf = BMP3_SPI_INTF;
//             (void)coines_config_spi_bus(COINES_SPI_BUS_0, COINES_SPI_SPEED_7_5_MHZ, COINES_SPI_MODE0);
//         }
//
//         coines_delay_msec(1000);
//
//         (void)coines_set_shuttleboard_vdd_vddio_config(3300, 3300);
//
//         coines_delay_msec(1000);
//
//         bmp3->delay_us = bmp3_delay_us;
//         bmp3->intf_ptr = &dev_addr;
//     }
//     else
//     {
//         rslt = BMP3_E_NULL_PTR;
//     }
//
//     return rslt;

    int8_t rslt = BMP3_OK;

    if (bmp3 != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMP3_I2C_INTF)
        {
            printf("NOTE: Configuring I2C mode for BMP...\n");
            dev_addr = BMP3_ADDR_I2C_PRIM;  // 0x76 i2c dev address (primary - SDO to GND)
            bmp3->read = bmp3_i2c_read;
            bmp3->write = bmp3_i2c_write;
            bmp3->intf = BMP3_I2C_INTF;
            bmp3->delay_us = bmp3_delay_us;
            bmp3->intf_ptr = &dev_addr;

            ace_init_i2c_bus();
        }
        /* Bus configuration : SPI */
        else if (intf == BMP3_SPI_INTF)
        {
            printf("ERROR: The ACE3.0 does not support SPI mode for the BMP!\n");
            rslt = BMP3_E_COMM_FAIL;
        }        
    }
    else
    {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;

}

void ace_deinit_i2c_bus()
{
    /* breadn-note: replaced with custom functionality */
    // (void)fflush(stdout);

    // (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);
    // coines_delay_msec(1000);

    // /* Coines interface reset */
    // coines_soft_reset();
    // coines_delay_msec(1000);
    // (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);

    i2c_deinit(i2c_default);
}

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