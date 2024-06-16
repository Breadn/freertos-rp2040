#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
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

/* Semaphores */
SemaphoreHandle_t sdSemaphore;

/******************************************************************************/
/*!         Static Function Declaration                                       */

static void initSensorHardware();
static void sampleTask(void *pvParameters);
static void computeTask(void *pvParameters);
static void storeTask(void *pvParameters);

static void errorLoop(const char* e_msg);

/******************************************************************************/
/*!            Macros                                        */

#define SECOND_US (uint32_t)1000000

/******************************************************************************/
/*!         Main application                                                  */

int main() {
    // init gpio pins
    stdio_init_all();

    // init sensor hardware
    initSensorHardware();

    // Semaphore creations
    sdSemaphore = xSemaphoreCreateMutex();
    if (sdSemaphore == NULL) {
        errorLoop("sensorDataSemphr: NORAM");
    }

    // Task creations
    xTaskCreate(computeTask, "computetask", 256, NULL, 1, NULL);
    xTaskCreate(storeTask, "storeTask", 256, NULL, 1, NULL);
    
    // Start rtos scheduler
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
    }

    busy_wait_us_32(SECOND_US);
    printf("*** END BMP5 INIT...\n");
}


static void storeTask(void *pvParameters) {
    int8_t rslt;
    struct bmp5_sensor_data data[BMP5_FIFO_P_FRAME_COUNT] = { { 0 } };   // 32 frames for pressure

    for (;;) {
        if (sdSemaphore != NULL) {
            if (xSemaphoreTake(sdSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE) {
                printf("Store task running...\n");
                vTaskDelay(100);
                rslt = get_fifo_data(&fifo, &dev, data);

                if (rslt == BMP5_OK) {
                    uint8_t i;
                    for (i=0; i<fifo.fifo_count; ++i) {
                        printf("StoreTask:\t%d\t%4.3f\n", i, data[0].pressure);
                    }
                    printf("Store task ran!\n");
                } else {
                    printf("StoreTask: Data not ready\n");
                }

                xSemaphoreGive(sdSemaphore);
            }
            else {
                printf("StoreTask: Could not obtain semaphore...\n");
            }
        }
        else {
            printf("StoreTask: Was semaphore created?\n");
        }
    }

    printf("WARNING: storeTask exit!\n");
    vTaskDelete(NULL);
}

static void computeTask(void *pvParameters) {
    int8_t rslt;
    struct bmp5_sensor_data data[BMP5_FIFO_P_FRAME_COUNT] = { { 0 } };   // 32 frames for pressure

    for (;;) {
        if (sdSemaphore != NULL) {
            if (xSemaphoreTake(sdSemaphore, (TickType_t) portMAX_DELAY) == pdTRUE) {
                printf("Compute task running...\n");
                vTaskDelay(100);
                rslt = get_fifo_data(&fifo, &dev, data);

                if (rslt == BMP5_OK) {
                    uint8_t i;
                    for (i=0; i<fifo.fifo_count; ++i) {
                        printf("ComputeTask:\t%d\t%4.3f\n", i, data[0].pressure);
                    }
                    printf("Compute task ran!\n");
                } else {
                    printf("ComputeTask: Data not ready\n");
                }
                
                xSemaphoreGive(sdSemaphore);
            }
            else {
                printf("ComputeTask: Could not obtain semaphore\n");
            }
        }
        else {
            printf("ComputeTask: Was semaphore created?\n");
        }
    }
    printf("WARNING: storeTask exit!\n");
    vTaskDelete(NULL);
}




static void errorLoop(const char* e_msg) {
    for (;;) {
        vTaskDelay(1000);
        printf(e_msg);
        printf("\n");
    }
}