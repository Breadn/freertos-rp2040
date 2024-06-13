#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

/* library includes 
    (visible from 'target_include_directories' in this dir's CMake file)
*/
#include "bmp5_internal.h"  // bmp5 internal API

/******************************************************************************/
/*!         Static Variable Declaration                                       */

struct bmp5_dev dev;
struct bmp5_fifo fifo;
struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

/******************************************************************************/
/*!         Static Function Declaration                                       */

static void initSensorHardware();
static void workTask(void *pvParameters);

/******************************************************************************/
/*!            Macros                                        */

#define SECOND_US (uint8_t)1000000

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
        sleep_us(SECOND_US);
        printf("Not enough RAM to start RTOS!\n");
    };

    return 0;
}


/******************************************************************************/
/*!         Function Definition                                               */

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
