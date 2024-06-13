#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

/* library includes */  // visible from 'target_include_directories' in this dir's CMake file
#include "bmp5.h"           // bmp5 api
#include "bmp5_common.h"    // bmp5 dev IO interface

/******************************************************************************/
/*!         Static Variable Declaration                                       */

struct bmp5_dev dev;
struct bmp5_fifo fifo;
struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations of the sensor.
 *
 *  @param[in,out] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev                   : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_norm_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor data.
 *
 *  @param[in] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev               : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

static void initSensorHardware();
static void workTask(void *pvParameters);

/*!
 *  @brief This internal API is used to set fifo configurations of the sensor.
 *
 *  @param[in,out] fifo : Structure instance of bmp5_fifo.
 *  @param[in] dev      : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_fifo_config(struct bmp5_fifo *fifo, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor fifo data.
 *
 *  @param[in] fifo : Structure instance of bmp5_fifo.
 *  @param[in] dev  : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
static int8_t get_fifo_data(struct bmp5_fifo *fifo, struct bmp5_dev *dev);

/******************************************************************************/
/*!            Macros                                        */

#define LOOP_COUNT                  UINT8_C(20)
#define BMP5_FIFO_DATA_BUFFER_SIZE  UINT8_C(96)
#define BMP5_FIFO_DATA_USER_LENGTH  UINT8_C(96)
#define BMP5_FIFO_P_T_FRAME_COUNT   UINT8_C(16)
#define BMP5_FIFO_T_FRAME_COUNT     UINT8_C(32)
#define BMP5_FIFO_P_FRAME_COUNT     UINT8_C(32)


/******************************************************************************/
/*!         Main application                                                  */

int main() {
    // init gpio pins
    stdio_init_all();

    // init sensor hardware
    initSensorHardware();

    xTaskCreate(workTask, "Work Task", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    for (;;) {
        sleep_us(1000000);
        printf("Not enough RAM to start RTOS!\n");
    };

    return 0;
}


// TODO: accept pointer to device to be init...or malloc device object and return pointer to that
static void initSensorHardware() {
    /* BMP5 init */
    int8_t rslt;

    /* Interface reference is given as a parameter
     * For I2C : BMP5_I2C_INTF
     * For SPI : BMP5_SPI_INTF
     */
    printf("> Initialize interface\n");
    rslt = bmp5_interface_init(&dev, BMP5_I2C_INTF);
    bmp5_error_codes_print_result("bmp5_interface_init", rslt);

    if (rslt == BMP5_OK)
    {
        printf("> Initialize sensor\n");
        rslt = bmp5_init(&dev);
        bmp5_error_codes_print_result("bmp5_init", rslt);

        if (rslt == BMP5_OK)
        {
            printf("> Configure sensor\n");
            rslt = set_fifo_config(&fifo, &dev);
            bmp5_error_codes_print_result("set_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            printf("> Get sensor data\n");
            rslt = get_fifo_data(&fifo, &dev);
            bmp5_error_codes_print_result("get_fifo_data", rslt);
        }
    }

    printf("*** END BMP5 INIT...\n");
}


static void workTask(void *pvParameters) {
    int8_t rslt;

    while (1) {
        vTaskDelay(1000);
        rslt = get_fifo_data(&fifo, &dev);
        bmp5_error_codes_print_result("get_fifo_data", rslt);
    }
}


static int8_t set_norm_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(osr_odr_press_cfg, dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 50Hz */
            osr_odr_press_cfg->odr = BMP5_ODR_50_HZ;

            /* Enable pressure */
            osr_odr_press_cfg->press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            osr_odr_press_cfg->osr_t = BMP5_OVERSAMPLING_64X;
            osr_odr_press_cfg->osr_p = BMP5_OVERSAMPLING_4X;

            rslt = bmp5_set_osr_odr_press_config(osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;
            set_iir_cfg.shdw_set_iir_t = BMP5_ENABLE;
            set_iir_cfg.shdw_set_iir_p = BMP5_ENABLE;

            rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);
            bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.drdy_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /* Set powermode as normal */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode", rslt);
    }

    return rslt;
}

static int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev)
{
    int8_t rslt = 0;
    uint8_t idx = 0;
    uint8_t int_status;
    struct bmp5_sensor_data sensor_data;

    printf("\nOutput :\n\n");
    printf("Data, Pressure (Pa), Temperature (deg C)\n");

    while (idx < 50)
    {
        rslt = bmp5_get_interrupt_status(&int_status, dev);
        bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);

        if (int_status & BMP5_INT_ASSERTED_DRDY)
        {
            rslt = bmp5_get_sensor_data(&sensor_data, osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_get_sensor_data", rslt);

            if (rslt == BMP5_OK)
            {
#ifdef BMP5_USE_FIXED_POINT
                printf("%d, %lu, %ld\n", idx, (long unsigned int)sensor_data.pressure,
                       (long int)sensor_data.temperature);
#else
                printf("%2.2f, %2.2f\n", sensor_data.pressure, sensor_data.temperature);
#endif

                idx++;
            }
        }
    }

    return rslt;
}

static int8_t set_fifo_config(struct bmp5_fifo *fifo, struct bmp5_dev *dev)
{
    int8_t rslt;
    struct bmp5_iir_config set_iir_cfg;
    struct bmp5_osr_odr_press_config osr_odr_press_cfg;
    struct bmp5_int_source_select int_source_select;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, dev);
    bmp5_error_codes_print_result("bmp5_set_power_mode1", rslt);

    if (rslt == BMP5_OK)
    {
        /* Get default odr */
        rslt = bmp5_get_osr_odr_press_config(&osr_odr_press_cfg, dev);
        bmp5_error_codes_print_result("bmp5_get_osr_odr_press_config", rslt);

        if (rslt == BMP5_OK)
        {
            /* Set ODR as 140Hz */
            osr_odr_press_cfg.odr = BMP5_ODR_140_HZ;

            /* Enable pressure */

            /* NOTE: If temperature data only is required then pressure enable(press_en) is not required
             * or can be disabled. */
            osr_odr_press_cfg.press_en = BMP5_ENABLE;

            /* Set Over-sampling rate with respect to odr */
            // Note: OSR_Temp = 2x and OSR_Pres = 8x has max ODR of 140Hz
            osr_odr_press_cfg.osr_t = BMP5_OVERSAMPLING_2X;
            osr_odr_press_cfg.osr_p = BMP5_OVERSAMPLING_8X;

            rslt = bmp5_set_osr_odr_press_config(&osr_odr_press_cfg, dev);
            bmp5_error_codes_print_result("bmp5_set_osr_odr_press_config", rslt);

            /* configure iir low-pass filter */
            if (rslt == BMP5_OK)
            {
                set_iir_cfg.set_iir_t = BMP5_IIR_FILTER_COEFF_1;
                set_iir_cfg.set_iir_p = BMP5_IIR_FILTER_COEFF_1;

                rslt = bmp5_set_iir_config(&set_iir_cfg, dev);
                bmp5_error_codes_print_result("bmp5_set_iir_config", rslt);
            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_get_fifo_configuration(fifo, dev);
            bmp5_error_codes_print_result("bmp5_get_fifo_configuration", rslt);

            if (rslt == BMP5_OK)
            {
                fifo->mode = BMP5_FIFO_MODE_STREAMING;

                /* Frame selection can be used to select data frames,
                 * pressure data only(32 frames) - BMP5_FIFO_PRESSURE_DATA
                 * temperature data only(32 frames) - BMP5_FIFO_TEMPERATURE_DATA
                 * both pressure and temperature data(16 frames) - BMP5_FIFO_PRESS_TEMP_DATA
                 * Here, both pressure and temperature data is selected(BMP5_FIFO_PRESS_TEMP_DATA)
                 */
                fifo->frame_sel = BMP5_FIFO_PRESSURE_DATA;  // only enable pressure (0b10)

                fifo->dec_sel = BMP5_FIFO_NO_DOWNSAMPLING;
                fifo->set_fifo_iir_t = BMP5_ENABLE;
                fifo->set_fifo_iir_p = BMP5_ENABLE;

                rslt = bmp5_set_fifo_configuration(fifo, dev);
                bmp5_error_codes_print_result("bmp5_set_fifo_configuration", rslt);
            }
        }

        if (rslt == BMP5_OK)
        {
            rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL, BMP5_INTR_ENABLE, dev);
            bmp5_error_codes_print_result("bmp5_configure_interrupt", rslt);

            if (rslt == BMP5_OK)
            {
                /* Note : Select INT_SOURCE after configuring interrupt */
                int_source_select.fifo_full_en = BMP5_ENABLE;
                rslt = bmp5_int_source_select(&int_source_select, dev);
                bmp5_error_codes_print_result("bmp5_int_source_select", rslt);
            }
        }

        /*
         * FIFO example can be executed on,
         * normal mode - BMP5_POWERMODE_NORMAL
         * continuous mode - BMP5_POWERMODE_CONTINOUS
         * Here, used normal mode (BMP5_POWERMODE_NORMAL)
         */
        rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, dev);
        bmp5_error_codes_print_result("bmp5_set_power_mode2", rslt);
    }

    return rslt;
}

static int8_t get_fifo_data(struct bmp5_fifo *fifo, struct bmp5_dev *dev)
{
    int8_t rslt = 0;
    uint8_t idx = 0;
    uint8_t loop = 0;
    uint8_t int_status;
    uint8_t fifo_buffer[BMP5_FIFO_DATA_BUFFER_SIZE];
    struct bmp5_sensor_data sensor_data[BMP5_FIFO_P_FRAME_COUNT] = { { 0 } };   // 32 frames for pressure

    printf("\nOutput :\n");

    while (loop < LOOP_COUNT)
    {
        rslt = bmp5_get_interrupt_status(&int_status, dev);
        bmp5_error_codes_print_result("bmp5_get_interrupt_status", rslt);

        if (int_status & BMP5_INT_ASSERTED_FIFO_FULL)
        {
            fifo->length = BMP5_FIFO_DATA_USER_LENGTH;
            fifo->data = fifo_buffer;

            printf("\nIteration: %d\n", loop);
            printf("Each fifo frame contains 6 bytes of data\n");
            printf("Fifo data bytes requested: %d\n", fifo->length);

            rslt = bmp5_get_fifo_data(fifo, dev);
            bmp5_error_codes_print_result("bmp5_get_fifo_data", rslt);

            printf("Fifo data bytes available: %d\n", fifo->length);
            printf("Fifo frames available: %d\n", fifo->fifo_count);

            if (rslt == BMP5_OK)
            {
                rslt = bmp5_extract_fifo_data(fifo, sensor_data);
                bmp5_error_codes_print_result("bmp5_extract_fifo_data", rslt);

                if (rslt == BMP5_OK)
                {
                    printf("\nData, Pressure (Pa), Temperature (deg C)\n");

                    for (idx = 0; idx < fifo->fifo_count; idx++)
                    {
#ifdef BMP5_USE_FIXED_POINT
                        printf("%d, %lu, %ld\n",
                               idx,
                               (long unsigned int)sensor_data[idx].pressure,
                               (long int)sensor_data[idx].temperature);
#else
                        printf("%d, %f, %f\n", idx, sensor_data[idx].pressure, sensor_data[idx].temperature);
#endif
                    }
                }

                loop++;
            }
        }
    }

    return rslt;
}