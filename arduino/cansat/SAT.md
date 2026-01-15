# Satellite Info

## Info
The CanSat is connected to a APC220 Wireless Antenna (with an equivalent on the ground), and data is collected with a
GY-91 sensor (which contains a BMP-280 though we have spare BMP-280 only boards)

GY-91 is connected through hardware i2c
Radio is connected through serial
Datalogger is software SPI

## Arduino Pin Mapping
| Arduino Pin | Chip Pin |
| -----: | :----- |
| 7  | APC220 1 (SET) |
| 8  | APC220 3 (TXD) |
| 9  | APC220 4 (RXD) |
| 10 | Datalogger 2 (MOSI) |
| 11 | Datalogger 3 (MISO) |
| 12 | Datalogger 4 (SCK) |
| A4 | GY-91 4 (SDA) |
| A5 | GY-91 5 (SCL) |

## Chip Pin Mapping

### APC220
| Pin | Label | Notes |
| -----: | :-----: | :----- |
| 1 | SET | Configuration set pin, set low while configuring, connected to Arduino 7 |
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
| 3 | SDO | n/c |
| 4 | SDA | Serial data pin, connected to Arduino A4 (hardware i2c) |
| 5 | SCL | Serial clock pin, connected to Arduino A5 (hardware i2c) |
| 6 | GND | Ground pin, connected to Arduino GND |
| 7 | 3v3 | 3v3 pin, connected to Arduino 3v3 |
| 8 | VIN | uhh don't connect I think? |

### Datalogger
| Pin | Label | Notes |
| -----: | :-----: | :----- |
| 1 | GND | Ground pin, connected to Arduino GND |
| 2 | MOSI | Master out slave in pin, connected to Arduino 10 |
| 3 | MISO | Master in slave out pin, connected to Arduino 11 |
| 4 | SCK | Clock pin, connected to Arduino 12 |
| 5 | 3v3 | 3v3 pin, connected to Arduino 3v3 |
| 6 | RST | Reset pin? Don't connect probably |

## Radio Packets

### Idle Packet (SAT -> BASE)
Sent by the satellite while waiting for the ground startup packet. The CanSat at this point is completely initialised
and is ready to start.

| Size | Part | Example |
| -----: | :----- | :-----: |
| `1` | Prefix | `0xAA` |
| `1` | Error | See error table below |
| `8` | Version Name | A null-terminated 7 letter SkyWing name, e.g. `0x80 0x69 0x82 0x73 0x76 0x32 0x32 0x00` "`PERIL  \x00`" |

### Data Packet (SAT -> BASE)
Sent as often as possible by the satellite while in the air. Don't log important data using these since they could cut
out, only use the data from the datalogger.

| Size | Part | Example |
| -----: | :----- | :-----: |
| `1` | Prefix | `0xAB` |
| `2` | Checksum | A CRC-16 checksum on the rest of the packet, e.g. `0x12 0x23` |
| `4`, `4`, `4` | Accelerometer | 3 floats of accelerometer data, in milliGs |
| `4`, `4`, `4` | Gyroscope | 3 floats of gyroscope data, in degrees per second |
| `4`, `4`, `4` | Magnetometer | 3 floats of magnetometer data, in degrees per second |
| `4`, `4`, `4` | Angle Data | 3 floats of angle data, yaw, pitch, roll, in degrees |
| `4` | Temperature | 1 float of temperature data, in celsius |
| `4` | Count | The amount of milliseconds since the CanSat turned on as an int |
| `4` | Refresh rate | The refresh rate of the IMU as a float |

### Launch Packet (SAT <- BASE)
Sent by the base when the satellite should start collecting data. Idle packets will stop being sent and data packets
will start.

| Size | Part | Example |
| -----: | :----- | :-----: |
| `1` | Prefix | `0xBA` |

### Radio Config Packet (SAT <- BASE)
Sent by the base to configure radio signals.

| Size | Part | Example |
| -----: | :----- | :-----: |
| `1` | Prefix | `0xBB` |
| `?` | Null-terminated config string (see bottom) | `"WR 433900 3 9 3 0"` |

### Land packet (SAT <- BASE)
Sent by the base when the satellite should stop collecting data.

| Size | Part | Example |
| -----: | :----- | :-----: |
| `1` | Prefix | `0xBC` |

## References

### Error Codes
| Code | Error |
| 0xFF | No error |
| 0x01 | IMU setup failed |
| 0x02 | Logger setup failed |
| 0x03 | Radio setup failed |

### Radio Config String
`WR <frequency> <rf data rate> <output power> <uart rate> <series checkout>`
Possible values:
    - Frequency (KHz), (434MHz = 434000 etc)
    - RF Data Rate: 1, 2, 3 and 4 refer to 2400, 4800, 9600 and 19200bps
    - Output Power: 0 to 9, 9 meaning 13dBm(20mW)
    - UART Rate: 0, 1, 2, 3, 4, 5 and 6 refer to 1200, 2400, 4800, 9600, 19200, 38400, 57600bps
    - Series Checkout: 0 means no check, 1 means even parity, 2 means odd parity.

Ours is configured with `"WR 433900 1 9 1 0"` (433.9MHz, 2400bps, 13dBm (whatever this means), 2400bps, no check)
