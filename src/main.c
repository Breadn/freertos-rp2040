#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    
    for (;;) {
        sleep_us(3000);
        printf("hello, world...!\n");
    }

    return 0;
}
