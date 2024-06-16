# freertos-rp2040

## ACE 3.0 Development

 because i need/want/should probably figure this stuff out


## Known Issues
* Hardwired SD module has CS pulled up high, disconnects from SPI bus
* Lack of external pullups to I2C and SPI buses
* BMI accel + gyro interrupts are not wired together for data synchronization
* Missing exposed UART header pins
