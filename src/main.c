#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdio.h>
#include "pico/stdlib.h"

/* library includes 
    (visible from 'target_include_directories' in this dir's CMake file)
*/


/******************************************************************************/
/*!            Defines                                                        */

#define SECOND_US       (uint32_t)  1000000
#define SECOND_MS       (uint32_t)  1000
#define DEBUG_PRINT_CT  (uint8_t)   5
#define TASK_STACK_SIZE (uint8_t)   200
#define NUM_TASKS                   4

/******************************************************************************/
/*!         Global Synchronization Variables                                  */
SemaphoreHandle_t xTestMutex;
SemaphoreHandle_t xBeginSemaphore;  // used to synchronize main task's children




/******************************************************************************/
/*!         Function Declarations                                             */



static void errorLoop(const char* e_msg);

static void LittleSemaphoreExample();
static void xLittleSemaphoreMainTask(void *pvParameters);

static void workerThreadA(void *pvParameters);
static void workerThreadB(void *pvParameters);
static void workerThreadC(void *pvParameters);
static void workerThreadD(void *pvParameters);

extern int RunRendezvousExample();


/******************************************************************************/
/*!         Main application                                                  */

int main() {
    // Initializations
    stdio_init_all();
    int begin;
    scanf("%x", (unsigned int *)&begin);  // wait for reply before beginning

    xBeginSemaphore = xSemaphoreCreateCounting(1, 1);
    if (xBeginSemaphore == NULL)
        errorLoop("Could not create xBeginMutex");
    
    // Create Main task
    if (begin != 0) {
        printf("Creating Main Task...\n");
        TaskHandle_t xMainTask;
        xMainTask = (TaskHandle_t)xTaskCreate(
            xLittleSemaphoreMainTask,
            "MainThread",
            PICO_STACK_SIZE,
            (void *) NULL,
            tskIDLE_PRIORITY,
            &xMainTask);

        // Start rtos scheduler
        printf("Starting Scheduler!\n");
        vTaskStartScheduler();

        errorLoop("Not enough RAM to start RTOS!");
    }
    printf("Aborting program execution...terminating\n");

    return 0;
}


/******************************************************************************/
/*!         Function Definitions                                              */

static void errorLoop(const char* e_msg) {
    for (;;) {
        sleep_us(SECOND_US);
        printf(e_msg);
        printf("\n");
    }
}

static void xLittleSemaphoreMainTask(void *pvParameters) {

    BaseType_t xReturned;
    TaskHandle_t xHandles[NUM_TASKS];
    bool spawnRslt = true;

    // Hold begin mutex
    xSemaphoreTake(xBeginSemaphore, portMAX_DELAY);

    // Init synchronization globals
    xTestMutex = xSemaphoreCreateMutex();
    if (xTestMutex == NULL)
        errorLoop("Could not initialize semaphores\n");

    // Spawn children task to run examples
    for (int i=0; i<NUM_TASKS; ++i) {
        char name[20];
        sprintf(name, "Thread%d", i);
        xReturned = xTaskCreate(
            LittleSemaphoreExample,
            name,
            PICO_STACK_SIZE,
            (void *)NULL,   // no parameter
            tskIDLE_PRIORITY,
            &xHandles[i]);
        
        if (xReturned != pdPASS) {
            spawnRslt = false;
            break;
        }
        printf("Created child task %s\n", name);
    }

    if (!spawnRslt) {
        printf("Failed to create children tasks!\n");
        for (int i=0; xHandles[i]; ++i) {
            vTaskDelete(xHandles[i]);
        }
    }
    else {
        vTaskDelay(100);
        printf("Starting in 3...\n");
        vTaskDelay(SECOND_MS*3);
        printf("Starting!\n");
        xSemaphoreGive(xBeginSemaphore);    // begin children
        vTaskSuspend(NULL); // parent is done
    }

    errorLoop("Did not successfully spawn children task");
}

static void LittleSemaphoreExample() {
    printf("Child waiting...\n");
    xSemaphoreTake(xBeginSemaphore, portMAX_DELAY);
    xSemaphoreGive(xBeginSemaphore);
    RunRendezvousExample();
    errorLoop("Invalid state reached");
}