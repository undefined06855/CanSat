/*
 * CanSat data logging + communication
 *
 * Arduino A5  -  GY-91 pin 5 (clock)
 * Arduino A4  -  GY-91 pin 4 (data)
 * Arduino 7   -  APC220 pin 1 (SET)
 * Arduino 8   -  APC220 pin 3 (TX) (with p/u resistor to 3.3v)
 * Arduino 9   -  APC220 pin 4 (RX) (with p/u resistor to 3.3v)
 * Arduino 10  -  Datalogger (MOSI)
 * Arduino 11  -  Datalogger (MISO)
 * Arduino 12  -  Datalogger (SCK)
 * Note: Datalogger has no CS pin!
 *
 * Requirements:
 *  - https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library (install as zip)
 *  - https://github.com/MajenkoLibraries/SoftSPI (also install as zip)
 *  - being cool enough
 *  
 */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU9250.h>
#include <quaternionFilters.h>
#include <SoftSPI.h>
#include <SoftwareSerial.h>

#define VERSION_STRING "KESTREL"

MPU9250 imu(MPU9250_ADDRESS_AD0, Wire, 400000);
SoftSPI logger(10, 11, 12);
SoftwareSerial radio(8, 9);

struct ImuState {
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float yaw, pitch, roll;
    float temperature;
    uint32_t count;
    float refreshRate;
};

struct ImuPacket {
    uint8_t prefix;
    uint16_t checksum;
    ImuState data;
};

struct IdlePacket {
    uint8_t prefix;
    char name[8];
};

ImuState imu_lastState = { 0 };
uint8_t* radio_buffer;

// https://github.com/Schildkroet/CRC/blob/master/CRC.c#L106
uint16_t checksum_calculate(const uint8_t* data, int size) {
    uint16_t checksum = 0xffff;

    for (uint16_t index = 0u; index < size; index++) {
        checksum ^= (data[index] << 8);

        for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
            if (checksum & (1 << 15)) {
                checksum = (checksum << 1) ^ 0x1021;
            } else {
                checksum = (checksum << 1);
            }
        }
    }
    
    return checksum ^ 0xff;
}

// initialises the IMU (GY-91)
// https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250BasicAHRS_I2C/MPU9250BasicAHRS_I2C.ino
bool imu_setup() {
    Serial.println("-- IMU SETUP BEGIN --");

    Wire.begin();

    // gyro setup

    Serial.println("imu gyro setup...");
    byte c = imu.readByte(MPU9250_ADDRESS_AD0, WHO_AM_I_MPU9250);
    Serial.print("imu gyro whoami (0x71): 0x"); Serial.println(c, HEX);
    if (c != 0x71) return false;

    Serial.println("imu gyro self test...");

    imu.MPU9250SelfTest(imu.selfTest);
    Serial.print("x-axis self test: acceleration trim within: ");
    Serial.print(imu.selfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within: ");
    Serial.print(imu.selfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within: ");
    Serial.print(imu.selfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within: ");
    Serial.print(imu.selfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within: ");
    Serial.print(imu.selfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within: ");
    Serial.print(imu.selfTest[5], 1); Serial.println("% of factory value");

    Serial.println("imu gyro calibration...");
    imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);

    Serial.println("imu gyro init...");
    imu.initMPU9250();

    // magnetometer setup

    Serial.println("imu magnetometer setup...");
    byte d = imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("imu magnetometer whoami (0x48): 0x"); Serial.println(d, HEX);
    if (d != 0x48) return false;

    Serial.println("imu magnetometer factory values:");
    Serial.print("x-axis factory sensitivity adjustment value: ");
    Serial.println(imu.factoryMagCalibration[0], 2);
    Serial.print("y-axis factory sensitivity adjustment value: ");
    Serial.println(imu.factoryMagCalibration[1], 2);
    Serial.print("z-axis factory sensitivity adjustment value: ");
    Serial.println(imu.factoryMagCalibration[2], 2);

    Serial.println("imu magnetometer init...");
    imu.initAK8963(imu.factoryMagCalibration);

    Serial.println("imu get sensor resolutions (wait 19 seconds)...");
    imu.getAres();
    imu.getGres();
    imu.getMres();

    imu.magCalMPU9250(imu.magBias, imu.magScale); // delays 4s, gathers 15s of data
    Serial.println("imu magnetometer mag biases (mG)");
    Serial.println(imu.magBias[0]);
    Serial.println(imu.magBias[1]);
    Serial.println(imu.magBias[2]);

    Serial.println("imu magnetometer mag scale (mG)");
    Serial.println(imu.magScale[0]);
    Serial.println(imu.magScale[1]);
    Serial.println(imu.magScale[2]);

    Serial.println("-- IMU SETUP COMPLETE --");
    return true;
}

void imu_read() {
    int status = imu.readByte(MPU9250_ADDRESS_AD0, INT_STATUS);
    if (~status & 0x01) return; // bit 1 is low, no data right now

    // you know okay so im not entirely sure why this is doing the thing it is
    // but the example
    // https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250BasicAHRS_I2C/MPU9250BasicAHRS_I2C.ino
    // does this

    imu.readAccelData(imu.accelCount);
    imu.ax = imu.accelCount[0] * imu.aRes; // - imu.accelBias[0]; // DO WE NEED THIS?????
    imu.ay = imu.accelCount[1] * imu.aRes;
    imu.az = imu.accelCount[2] * imu.aRes;

    imu.readGyroData(imu.gyroCount);
    imu.gx = imu.gyroCount[0] * imu.gRes;
    imu.gy = imu.gyroCount[1] * imu.gRes;
    imu.gz = imu.gyroCount[2] * imu.gRes;

    imu.readMagData(imu.magCount);
    imu.mx = imu.magCount[0] * imu.mRes * imu.factoryMagCalibration[0] - imu.magBias[0];
    imu.my = imu.magCount[1] * imu.mRes * imu.factoryMagCalibration[1] - imu.magBias[1];
    imu.mz = imu.magCount[2] * imu.mRes * imu.factoryMagCalibration[2] - imu.magBias[2];

    imu.updateTime();

    MahonyQuaternionUpdate(
      imu.ax, imu.ay, imu.az,
      imu.gx * DEG_TO_RAD, imu.gy * DEG_TO_RAD, imu.gz * DEG_TO_RAD,
      imu.my, imu.mx, imu.mz, // note: x and y are swapped for the magnetometer
      imu.deltat
    );

    imu.tempCount = imu.readTempData();
    imu.temperature = imu.tempCount / 333.87f + 21.f;

    // need to convert quaternion to pitch/roll/yaw
    // https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250BasicAHRS_I2C/MPU9250BasicAHRS_I2C.ino#L437

    imu.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
              * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
              * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
              * *(getQ()+3)) * RAD_TO_DEG;
    imu.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                  * *(getQ()+2))) * RAD_TO_DEG;
    imu.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                  * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                  * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                  * *(getQ()+3)) * RAD_TO_DEG;

    imu.delt_t = millis() - imu.count;
    imu.count = millis();
    imu.sumCount = 0;
    imu.sum = 0;

    imu_lastState = {
        imu.ax * 1000, imu.ay * 1000, imu.az * 1000, // values in milliGs
        imu.gx, imu.gy, imu.gz, // values in degrees per second
        imu.mx, imu.my, imu.mz, // values in degrees per second
        imu.yaw, imu.pitch, imu.roll, // values in degrees
        imu.temperature, // values in celsius
        millis(),
        imu.sumCount / imu.sum
    };
}

// initialises the logger
bool logger_setup() {
    Serial.println("-- LOGGER SETUP BEGIN --");
  
    logger.begin();

    Serial.println("-- LOGGER SETUP COMPLETE --");
    return true;
}

void logger_sendData(const char* data) {
    logger_sendData("[ ");
    logger_sendData(String(millis()).c_str());
    logger_sendData("ms ]: ");
    logger_sendData(data);
    logger_sendData("\n");
    Serial.println(data);
}

// logs data to the sd card
void logger_sendDataRaw(const char* data) {
    int len = strlen(data);
    logger_sendBytes((uint8_t*)data, len);
}

void logger_sendBytes(const uint8_t* data, int size) {
    for (int i = 0; i < size; i++) {
        logger.transfer(data[i]);
    }
}

// initialises the radio (APC220)
bool radio_setup() {
    Serial.println("-- RADIO SETUP BEGIN --");

    radio.begin(2400);
    digitalWrite(7, HIGH);

    Serial.println("-- RADIO SETUP COMPLETE --");
    return true;
}

// sends data back to the ground
void radio_sendData(const char* data) {
    radio.print(data);
}

void radio_sendBytes(const uint8_t* data, int size) {
    for (int i = 0; i < size; i++) {
        radio.write(data[i]);
    }

    radio.flush();
}

void radio_sendLastIMUReading() {
    ImuPacket data = {
        0xAB,
        checksum_calculate((uint8_t*)&imu_lastState, sizeof(imu_lastState)),
        imu_lastState
    };

    radio_sendBytes((uint8_t*)&data, sizeof(ImuPacket));
}

void radio_sendIdlePacket() {
    IdlePacket data = {
        0xAA,
        VERSION_STRING
    };

    radio_sendBytes((uint8_t*)&data, sizeof(IdlePacket));
}

void* radio_readData() {
    int bytes = radio.available();
    if (bytes == 0) return NULL;

    radio_buffer[bytes + 1] = 0x00;

    for (int i = 0; i < bytes; i++) {
        radio_buffer[i] = radio.read();
    }

    return radio_buffer;
}

void setup() {
    Serial.begin(9600);

    radio_buffer = (uint8_t*)malloc(32);

    if (!imu_setup()) {
        Serial.println("IMU SETUP FAILED!");
        return;
    }

    if (!logger_setup()) {
        Serial.println("LOGGER SETUP FAILED!");
        return;
    }

    if (!radio_setup()) {
        Serial.println("RADIO SETUP FAILED!");
        return;
    }

    logger_sendData("SATELLITE VERSION " VERSION_STRING);
    radio_sendData(VERSION_STRING "\n" );
}

// 0 - waiting
// 1 - launched
int state = 0;

void loop() {
    uint8_t* data = (uint8_t*)radio_readData();

    if (data) {
        switch (data[0]) {
            case 0xBA: {
                logger_sendData("launch packet received");
                state = 1;
            } break;

            case 0xBB: {
                logger_sendData("radio setup packet received");
                digitalWrite(7, LOW);
                radio_sendData((char*)&data[1]);
                digitalWrite(7, HIGH);
            } break;
        }
    }

    switch (state) {
        case 0: {
            // send idle packet
            radio_sendIdlePacket();
        } break;

        case 1: {
            // send imu data
            imu_read();
            radio_sendLastIMUReading();
        } break;
    }
}
