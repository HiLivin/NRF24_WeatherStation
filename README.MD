# NRF24 Weather Station

The station works with up to 4 battery-powered transmitters, each with a DS18B20 temperature sensor.

The receiver displays:
- current date, time of sunrise, time of sunset, Moon phase
- temperature, humidity, pressure from the receiver's BME280 sensor
- temperatures from the transmitters
- battery voltage of each of the transmitters
- time since last reading from each of the transmitters

There is an adjustable freezing alarm (it turns on when at least one of the transmitter reading falls below the set temperature).

All the readings can be logged to a SD card.

## Hardware used
### Transmitter
- Arduino Pro Mini (3.3V/8MHz)
- NRF24L01+
- DS18B20 sensor
- 3xAA battery holder

### Receiver
- Arduino UNO with power supply
- NRF24L01+
- BME280 sensor
- DS3231 AT24C32 Real Time Clock
- PCF8574 I2C expansion board
- LCD2004 with turning knob and SD card reader (Ramps1.4 compatible display)