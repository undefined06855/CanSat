package can_io

import (
    "bufio"
    "bytes"
    "encoding/binary"
    "fmt"
    "io"
    "reflect"
    "runtime"
    "time"
    "sync"

    "github.com/sigurn/crc16"
    "github.com/tarm/serial"
)

var logError error
var logMutex sync.Mutex
var logBuffer []string
var reader *bufio.Reader
var writer *bufio.Writer
var inited bool

type Data struct {
    AccelX, AccelY, AccelZ []float32
    GyroX, GyroY, GyroZ    []float32
    MagX, MagY, MagZ       []float32
    Yaw, Pitch, Roll       []float32
    Temperature            []float32
    Pressure               []float32
    Count                  []uint32
}

type DataSnapshot struct {
    AccelX, AccelY, AccelZ float32
    GyroX, GyroY, GyroZ    float32
    MagX, MagY, MagZ       float32
    Yaw, Pitch, Roll       float32
    Temperature            float32
    Pressure               float32
    Count                  uint32
    Index uint32
}

var satData Data

func InitLogs() {
    logBuffer = make([]string, 0)

    satData.AccelX = make([]float32, 0, 8_000_000)
    satData.AccelY = make([]float32, 0, 8_000_000)
    satData.AccelZ = make([]float32, 0, 8_000_000)
    satData.GyroX = make([]float32, 0, 8_000_000)
    satData.GyroY = make([]float32, 0, 8_000_000)
    satData.GyroZ = make([]float32, 0, 8_000_000)
    satData.MagX = make([]float32, 0, 8_000_000)
    satData.MagY = make([]float32, 0, 8_000_000)
    satData.MagZ = make([]float32, 0, 8_000_000)
    satData.Yaw = make([]float32, 0, 8_000_000)
    satData.Pitch = make([]float32, 0, 8_000_000)
    satData.Roll = make([]float32, 0, 8_000_000)
    satData.Temperature = make([]float32, 0, 8_000_000)
    satData.Pressure = make([]float32, 0, 8_000_000)
    satData.Count = make([]uint32, 0, 8_000_000)

    var name string
    if runtime.GOOS == "windows" {
        name = "COM7"
    } else {
        name = "/dev/ttyUSB0"
    }

    inited = false

    connect := func() {
        fmt.Println("trying to connect....")

        config := &serial.Config{
            Name: name,
            Baud: 115200,
            ReadTimeout: time.Second,
        }

        port, err := serial.OpenPort(config)
        if err != nil {
            logError = err
            time.Sleep(time.Second * 2)
            return
        }

        writer = bufio.NewWriter(port)
        reader = bufio.NewReader(port)

        inited = true
    }

    readSingleLine := func() {
        line, err := reader.ReadString('\n')

        if err != nil && err != io.EOF && err != io.ErrClosedPipe {
            logError = err
            return
        }

        if err == io.ErrClosedPipe {
            // try to reconnect
            inited = false
            return
        }

        logMutex.Lock()
        logBuffer = append(logBuffer, line)
        logMutex.Unlock()

        parseLogLine(line)

        fmt.Println(line)
    }

    go func() {
        for {
            for !inited { connect(); continue }
            readSingleLine()
        }
    }()
}

var index int
func parseLogLine(line string) {
    index++
    satData.Pressure = append(satData.Pressure, float32(index*index)) // temp example data
}

var checksumTable = crc16.MakeTable(crc16.CRC16_MAXIM)
func calculateChecksum(data []byte) uint16 {
    return crc16.Checksum(data, checksumTable)
}

func byteifyStruct(v any) ([]byte, error) {
    val := reflect.ValueOf(v)
    if val.Kind() != reflect.Struct {
        return nil, fmt.Errorf("expected struct")
    }

    buf := new(bytes.Buffer)

    for i := 0; i < val.NumField(); i++ {
        field := val.Field(i)

        switch field.Kind() {

        case reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64,
            reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:

            err := binary.Write(buf, binary.LittleEndian, field.Interface())
            if err != nil {
                return nil, err
            }

        case reflect.String:
            buf.WriteString(field.String())
            buf.WriteByte('\x00')

        default:
            return nil, fmt.Errorf("unsupported field type: %s", field.Kind())
        }
    }

    return buf.Bytes(), nil
}

func SendData(prefix byte, data any) error {
    if logError != nil {
        return logError
    }

    if !inited {
        return nil
    }

    err := binary.Write(writer, binary.LittleEndian, byte(0xff))
    if err != nil {
        logError = err
        return err
    }

    err = binary.Write(writer, binary.LittleEndian, prefix)
    if err != nil {
        logError = err
        return err
    }

    bytes, err := byteifyStruct(data)
    if err != nil {
        logError = err
        return err
    }

    checksum := calculateChecksum(bytes)

    err = binary.Write(writer, binary.LittleEndian, checksum)
    if err != nil {
        logError = err
        return err
    }

    err = binary.Write(writer, binary.LittleEndian, bytes)
    if err != nil {
        logError = err
        return err
    }

    return writer.Flush()
}

func GetLogs() []string {
    logMutex.Lock()
    defer logMutex.Unlock()

    if !inited {
        if logError != nil {
            return []string{"there was an error bucko\n", logError.Error()}
        }

        return []string{"not initialised yet!\n"}
    }

    if logError != nil {
        return []string{logError.Error()}
    }

    // copy a slice this is funny
    // i guess it makes sense
    return append([]string(nil), logBuffer...)
}

func GetError() error {
    return logError
}

func GetData() Data {
    return satData
}

func GetDataSnapshot() *DataSnapshot {
    if len(satData.Count) == 0 {
        return nil
    }

    index := uint32(len(satData.Count) - 1)
    return &DataSnapshot{
        AccelX: satData.AccelX[index],
        AccelY: satData.AccelY[index],
        AccelZ: satData.AccelZ[index],
        GyroX: satData.GyroX[index],
        GyroY: satData.GyroY[index],
        GyroZ: satData.GyroZ[index],
        MagX: satData.MagX[index],
        MagY: satData.MagY[index],
        MagZ: satData.MagZ[index],
        Yaw: satData.Yaw[index],
        Pitch: satData.Pitch[index],
        Roll: satData.Roll[index],
        Temperature: satData.Temperature[index],
        Pressure: satData.Pressure[index],
        Count: satData.Count[index],
        Index: index,
    }
}
