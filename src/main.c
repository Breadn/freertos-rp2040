#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/stdlib.h"

#include <stdio.h>
#include <stdlib.h>

/* library includes 
    (visible from 'target_include_directories' in this dir's CMake file)
*/
#include "bmp5_internal.h"  // bmp5 internal API

/******************************************************************************/
/*!            Macros                                        */

#define SECOND_US       (uint32_t)  1000000
#define DEBUG_PRINT_CT  (uint8_t)   5

/******************************************************************************/
/*!         Static Variable Declaration                                       */

struct bmp5_dev dev;
struct bmp5_fifo fifo;
struct bmp5_osr_odr_press_config osr_odr_press_cfg = { 0 };

/* Semaphores */
SemaphoreHandle_t ct_Semaphore;    // compute task
SemaphoreHandle_t st_Semaphore;    // store task

/* Shared data */
float ct_buf[BMP5_FIFO_P_FRAME_COUNT];
float st_buf[BMP5_FIFO_P_FRAME_COUNT];

/******************************************************************************/
/*!         Static Function Declaration                                       */

static void initSensorHardware();
static void sampleTask(void *pvParameters);
static void computeTask(void *pvParameters);
static void storeTask(void *pvParameters);

static void errorLoop(const char* e_msg);

/******************************************************************************/
/*!         Main application                                                  */

int main() {
    // init gpio pins
    stdio_init_all();

    // init sensor hardware
    initSensorHardware();

    // Semaphore creations
    ct_Semaphore = xSemaphoreCreateMutex();
    st_Semaphore = xSemaphoreCreateMutex();
    if ((ct_Semaphore == NULL) || (st_Semaphore == NULL)) {
        errorLoop("Semaphore creation: NORAM");
    }

    // Task creations
    xTaskCreate(sampleTask, "sampleTask", 256, NULL, 2, NULL);
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


static void sampleTask(void *pvParameters) {
    // this task must be responsible for sampling fifo for data
    // then queueing/sharing data with other tasks to resolve
    // read duplication issue

    int8_t rslt;
    struct bmp5_sensor_data data[BMP5_FIFO_P_FRAME_COUNT] = { { 0 } };   // 32 frames for pressure

    for (;;) {
        printf("Sample task running...\n");
        vTaskDelay(500);
        rslt = get_fifo_data(&fifo, &dev, data);

        if (rslt == BMP5_OK) {
            uint8_t i;
            uint8_t didWriteFlag = 0;

            // copy data to shared buffers
            while (didWriteFlag != 0x3) {
                if (!(didWriteFlag & 0x1)) {
                    // ct_semaphore
                    if (ct_Semaphore != NULL) {
                        if (xSemaphoreTake(ct_Semaphore, (TickType_t) 10) == pdTRUE) {
                            printf("sampleTask: Writing ct_buf...\n");

                            // copy data to ct_buf
                            for (i=0; i<BMP5_FIFO_P_FRAME_COUNT && i<fifo.fifo_count; ++i)
                                ct_buf[i] = data[i].pressure;

                            xSemaphoreGive(ct_Semaphore);
                            didWriteFlag |= 0x1;
                        } else {
                            printf("sampleTask: could not write ct_buf, retrying...\n");
                        }
                    }
                    // ct_semaphore
                }
                if (!(didWriteFlag & 0x2)) {
                    // st_semphr
                    if (st_Semaphore != NULL) {
                        if (xSemaphoreTake(st_Semaphore, (TickType_t) 10) == pdTRUE) {
                            printf("sampleTask: Writing st_buf...\n");

                            // copy data to st_buf
                            for (i=0; i<BMP5_FIFO_P_FRAME_COUNT && i<fifo.fifo_count; ++i)
                                st_buf[i] = data[i].pressure;

                            xSemaphoreGive(st_Semaphore);
                            didWriteFlag |= 0x2;
                        } else {
                            printf("sampleTask: could not write st_buf, retrying...\n");
                        }
                    }
                    // st_semphr
                }

            }
        } else {
            printf("sampleTask: Data not ready\n");
        }
        printf("Sample task complete!\n\n");
    }

    printf("WARNING: sampleTask exit!\n");
    vTaskDelete(NULL);

}


static void computeTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(1000);
        printf("Compute task running...\n");
        // ct_semphr
        if (ct_Semaphore != NULL) {
            if (xSemaphoreTake(ct_Semaphore, (TickType_t) 10) == pdTRUE) {
                uint8_t i;

                // consume data
                printf("Compute task consume: ");
                for (i=0; i<DEBUG_PRINT_CT; ++i)
                    printf("\t%2.2f", ct_buf[i]);
                printf("\n");

                // generate random amount of busy time
                int randMlt = rand() % 5;
                vTaskDelay(100*randMlt);

                xSemaphoreGive(ct_Semaphore);
            } else {
                printf("computeTask: could not get semaphore\n");
            }
        }
        // ct_semphr
        printf("Compute task complete!\n\n");
    }
    printf("WARNING: computeTask exit!\n");
    vTaskDelete(NULL);
}


static void storeTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(1000);
        printf("Store task running...\n");
        // st_semphr
        if (st_Semaphore != NULL) {
            if (xSemaphoreTake(st_Semaphore, (TickType_t) 10) == pdTRUE) {
                uint8_t i;

                // consume data
                printf("Store task consume: ");
                for (i=0; i<DEBUG_PRINT_CT; ++i)
                    printf("\t%2.2f", st_buf[i]);
                printf("\n");

                // generate random amount of busy time
                int randMlt = rand() % 5;
                vTaskDelay(100*randMlt);

                xSemaphoreGive(st_Semaphore);
            } else {
                printf("storeTask: could not get semaphore\n");
            }
        }
        // st_semphr
        printf("Store task complete!\n\n");
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