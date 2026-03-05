package main

import (
	"bufio"
	"encoding/binary"
	"fmt"
	"io"
    "bytes"
    "reflect"

	"github.com/tarm/serial"
    "github.com/sigurn/crc16"
)

var logError error
var logBuffer []string
var reader *bufio.Reader
var writer *bufio.Writer

func initLogs() {
    logBuffer = make([]string, 0)

    config := &serial.Config{
        // Name: "/dev/ttyUSB0",
        Name: "COM4",
        Baud: 9600,
    }

    port, err := serial.OpenPort(config)
    if err != nil {
        logError = err
        return
    }

    writer = bufio.NewWriter(port)
    reader = bufio.NewReader(port)

    go func() {
        for {
            line, err := reader.ReadString('\n')

            if err != nil && err != io.EOF {
                logError = err
                continue
            }

            logBuffer = append(logBuffer, line)
            fmt.Println(line)
        }
    }()
}

var checksumTable = crc16.MakeTable(crc16.CRC16_MAXIM)
func calculateChecksum(data []byte) (uint16) {
    crc := crc16.Checksum(data, checksumTable)
    return crc
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
            // for all the normal shit
            case reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64,
                reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:

                err := binary.Write(buf, binary.LittleEndian, field.Interface())
                if err != nil {
                    return nil, err
                }

            // strings
            case reflect.String:
                buf.WriteString(field.String())
                buf.WriteByte('\x00')

            default:
                return nil, fmt.Errorf("unsupported field type: %s", field.Kind())
        }
    }

    return buf.Bytes(), nil
}

func sendData(prefix byte, data any) (error) {
    if logError != nil { return logError }

    // write begin byte
    err := binary.Write(writer, binary.LittleEndian, 0xff)
    if err != nil {
        logError = err
        return err
    }

    // then write prefix
    err = binary.Write(writer, binary.LittleEndian, prefix)
    if err != nil {
        logError = err
        return err
    }

    // convert to bytes
    bytes, err := byteifyStruct(data)
    if err != nil {
        logError = err
        return err
    }

    // then write the checksum
    checksum := calculateChecksum(bytes)
    err = binary.Write(writer, binary.LittleEndian, checksum)
    if err != nil {
        logError = err
        return err
    }

    // and write data
    err = binary.Write(writer, binary.LittleEndian, bytes)
    if err != nil {
        logError = err
        return err
    }

    return nil
}

func getLogs() []string {
    if logError != nil {
        return []string{ logError.Error() }
    }

    return logBuffer
}
