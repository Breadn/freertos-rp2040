#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

/* library includes */  // visible from 'target_include_directories' in this dir's CMake file
#include "bmp3.h"           // bmp3 api
#include "bmp3_common.h"    // bmp3 dev IO interface

static void initSensorHardware();
static void workTask(void *pvParameters);

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
    /* BMP3 init */
    int8_t rslt;
    uint16_t settings_sel;
    struct bmp3_dev dev;
    struct bmp3_settings settings = { 0 };

    printf("*** BEGIN BMP3 INIT...\n");

    /* Interface reference is given as a parameter
     *         For I2C : BMP3_I2C_INTF
     *         For SPI : BMP3_SPI_INTF
     */
    rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
    bmp3_check_rslt("bmp3_interface_init", rslt);

    rslt = bmp3_init(&dev);
    bmp3_check_rslt("bmp3_init", rslt);

    settings.int_settings.drdy_en = BMP3_ENABLE;
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                   BMP3_SEL_DRDY_EN;

    rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
    bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

    settings.op_mode = BMP3_MODE_NORMAL;
    rslt = bmp3_set_op_mode(&settings, &dev);
    bmp3_check_rslt("bmp3_set_op_mode", rslt);

    printf("*** END BMP3 INIT...\n");
}


static void workTask(void *pvParameters) {

    while (1) {
        vTaskDelay(1000);
        printf("hello, world!\n");
    }
}
