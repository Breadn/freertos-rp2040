#include "pico/stdlib.h"

#include "bmp5.h"           // bmp5 api
#include "bmp5_common.h"    // bmp5 dev IO interface

/******************************************************************************/
/*!            Macros                                                         */

#define BMP5_FIFO_DATA_BUFFER_SIZE  UINT8_C(96)
#define BMP5_FIFO_DATA_USER_LENGTH  UINT8_C(96)
#define BMP5_FIFO_P_T_FRAME_COUNT   UINT8_C(16)
#define BMP5_FIFO_T_FRAME_COUNT     UINT8_C(32)
#define BMP5_FIFO_P_FRAME_COUNT     UINT8_C(32)

#define BMP5_DATA_NOT_READY         UINT8_C(0x1)


/******************************************************************************/
/*!            Function Declaration                                           */

/*!
 *  @brief This internal API is used to set configurations of the sensor.
 *
 *  @param[in,out] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev                   : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
int8_t set_norm_config(struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor data.
 *
 *  @param[in] osr_odr_press_cfg : Structure instance of bmp5_osr_odr_press_config
 *  @param[in] dev               : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
int8_t get_sensor_data(const struct bmp5_osr_odr_press_config *osr_odr_press_cfg, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to set fifo configurations of the sensor.
 *
 *  @param[in,out] fifo : Structure instance of bmp5_fifo.
 *  @param[in] dev      : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
int8_t set_fifo_config(struct bmp5_fifo *fifo, struct bmp5_dev *dev);

/*!
 *  @brief This internal API is used to get sensor fifo data.
 *
 *  @param[in] fifo : Structure instance of bmp5_fifo.
 *  @param[in] dev  : Structure instance of bmp5_dev.
 *
 *  @return Status of execution.
 */
int8_t get_fifo_data(struct bmp5_fifo *fifo, struct bmp5_dev *dev, struct bmp5_sensor_data sensor_data[]);