# freertos-rp2040

## ACE 3.0 Development

Currently developing on Raspberry Pi Pico W instead of original ACE 3.0


## Known ACE 3.0 Issues
* Hardwired SD module has CS pulled up high, disconnects from SPI bus
* Lack of external pullups to I2C and SPI buses
* BMI accel + gyro interrupts are not wired together for data synchronization
* Missing exposed UART header pins
