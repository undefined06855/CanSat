# Satellite Info

## Info
The CanSat is connected to a APC220 Wireless Antenna (with an equivalent on the ground), and data is collected with a GY-91 sensor (which contains a BMP280 though we have spare BMP-280 only boards)

BMP280 is through software SPI not i2c
APC220 is just through tty or whatever

## Arduino Pin Mapping
| Arduino Pin | Chip Pin |
| -----: | :----- |
| 10 | BMP280 1 |
| 11 | BMP280 4 |
| 12 | BMP280 3 |
| 13 | BMP280 5 |
| 8 | APC220 3 |
| 9 | APC220 4 |

## Chip Pin Mapping

### APC220
| Pin | Label | Notes |
| -----: | :-----: | :----- |
| 1 | SET | n/c, (but connect to Arduino 13 when need to configure) |
| 2 | AUX | n/c |
| 3 | TXD | TX pin, connected to Arduino 8 |
| 4 | RXD | RX pin, connected to Arduino 9 |
| 5 | EN  | n/c |
| 6 | VCC | 3v3 pin, connected to Arduino 3v3 |
| 7 | GND | Ground pin, connected to Arduino GND |

### GY-91
| Pin | Label | Notes |
| -----: | :-----: | :----- |
| 1 | CSB | Chip Select pin, connected to Arduino 10 |
| 2 | n/c | n/c |
| 3 | SDO* | MISO pin, connected to Arduino 12 |
| 4 | SDA* | MOSI pin, connected to Arduino 11 |
| 5 | SCL | Clock pin, connected to Arduino 13 |
| 6 | GND | Ground pin, connected to Arduino GND |
| 7 | 3v3 | 3v3 pin, connected to Arduino 3v3 |
| 8 | VIN | ????, just connect to 3v3 |

\*Pin labels are for I2C or something, since we are using software SPI we are not using these labels.
