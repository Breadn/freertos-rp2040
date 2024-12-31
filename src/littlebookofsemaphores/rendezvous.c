#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pico/stdlib.h"

#include <stdio.h>

/* Declare Global Synchronization variables */
extern SemaphoreHandle_t xTestMutex;

int count = 0;

void RunRendezvousExample() {
    while (true) {
        xSemaphoreTake(xTestMutex, portMAX_DELAY);
        count += 1;
        printf("%d\n", count);
        xSemaphoreGive(xTestMutex);
        vTaskDelay(100);
    }
}