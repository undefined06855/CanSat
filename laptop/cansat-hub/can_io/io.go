package can_io

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"fmt"
	"io"
	"os"
	"reflect"
	"runtime"
	"strings"
	"sync"
	"time"

	"github.com/tarm/serial"
)

var logError error
var logMutex sync.Mutex
var logBuffer []string
var dataMutex sync.RWMutex
var stateMutex sync.RWMutex
var sendMutex sync.Mutex
var statsMutex sync.RWMutex
var reader *bufio.Reader
var writer *bufio.Writer
var inited bool
var activePort string

type PacketStats struct {
	IdlePackets   uint64
	ImuPackets    uint64
	ChecksumDrops uint64
	UnknownPacket uint64
	CommandsSent  uint64
	Reconnects    uint64
}

type Data struct {
	AccelX, AccelY, AccelZ []float32
	GyroX, GyroY, GyroZ    []float32
	MagX, MagY, MagZ       []float32
	Yaw, Pitch, Roll       []float32
	Temperature            []float32
	RefreshRate            []float32
	CountF                 []float32
	Count                  []uint32
}

type DataSnapshot struct {
	AccelX, AccelY, AccelZ float32
	GyroX, GyroY, GyroZ    float32
	MagX, MagY, MagZ       float32
	Yaw, Pitch, Roll       float32
	Temperature            float32
	RefreshRate            float32
	Count                  uint32
	Index                  uint32
}

var satData Data
var satStats PacketStats

const (
	packetBeginLow  = 0x0B
	packetBeginHigh = 0xB0

	packetIdle = 0xAA
	packetIMU  = 0xAB

	idlePayloadSize = 1 + 8
	imuPayloadSize  = 2 + 60
)

type imuStateWire struct {
	AccelX, AccelY, AccelZ float32
	GyroX, GyroY, GyroZ    float32
	MagX, MagY, MagZ       float32
	Yaw, Pitch, Roll       float32
	Temperature            float32
	Count                  uint32
	RefreshRate            float32
}

func appendLogf(format string, args ...any) {
	logMutex.Lock()
	defer logMutex.Unlock()

	logBuffer = append(logBuffer, fmt.Sprintf(format, args...))
	if len(logBuffer) > 2000 {
		logBuffer = logBuffer[len(logBuffer)-2000:]
	}
}

func setLogError(err error) {
	logMutex.Lock()
	logError = err
	logMutex.Unlock()
}

func setInited(v bool) {
	stateMutex.Lock()
	inited = v
	stateMutex.Unlock()
}

func getInited() bool {
	stateMutex.RLock()
	defer stateMutex.RUnlock()
	return inited
}

func setActivePort(name string) {
	stateMutex.Lock()
	activePort = name
	stateMutex.Unlock()
}

func getActivePort() string {
	stateMutex.RLock()
	defer stateMutex.RUnlock()
	return activePort
}

func updateStats(fn func(*PacketStats)) {
	statsMutex.Lock()
	defer statsMutex.Unlock()
	fn(&satStats)
}

func candidatePorts() []string {
	out := make([]string, 0, 10)
	env := strings.TrimSpace(os.Getenv("CANSAT_PORT"))
	if env != "" {
		out = append(out, env)
	}

	if runtime.GOOS == "windows" {
		out = append(out, "COM4", "COM5", "COM6", "COM7")
	} else {
		out = append(out,
			"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3",
			"/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyACM3",
		)
	}

	return out
}

func getLogError() error {
	logMutex.Lock()
	defer logMutex.Unlock()
	return logError
}

// CRC implementation kept identical to the Arduino transmitter.
func checksumCalculate(data []byte) uint16 {
	checksum := uint16(0xFFFF)

	for _, b := range data {
		checksum ^= uint16(b) << 8

		for bit := 0; bit < 8; bit++ {
			if checksum&(1<<15) != 0 {
				checksum = (checksum << 1) ^ 0x1021
			} else {
				checksum <<= 1
			}
		}
	}

	return checksum ^ 0x00FF
}

func appendIMU(state imuStateWire) {
	dataMutex.Lock()
	defer dataMutex.Unlock()

	satData.AccelX = append(satData.AccelX, state.AccelX)
	satData.AccelY = append(satData.AccelY, state.AccelY)
	satData.AccelZ = append(satData.AccelZ, state.AccelZ)
	satData.GyroX = append(satData.GyroX, state.GyroX)
	satData.GyroY = append(satData.GyroY, state.GyroY)
	satData.GyroZ = append(satData.GyroZ, state.GyroZ)
	satData.MagX = append(satData.MagX, state.MagX)
	satData.MagY = append(satData.MagY, state.MagY)
	satData.MagZ = append(satData.MagZ, state.MagZ)
	satData.Yaw = append(satData.Yaw, state.Yaw)
	satData.Pitch = append(satData.Pitch, state.Pitch)
	satData.Roll = append(satData.Roll, state.Roll)
	satData.Temperature = append(satData.Temperature, state.Temperature)
	satData.RefreshRate = append(satData.RefreshRate, state.RefreshRate)
	satData.CountF = append(satData.CountF, float32(state.Count))
	satData.Count = append(satData.Count, state.Count)
}

func readExactly(n int) ([]byte, error) {
	buf := make([]byte, n)
	_, err := io.ReadFull(reader, buf)
	return buf, err
}

func readAndParsePacket() error {
	for {
		b, err := reader.ReadByte()
		if err != nil {
			return err
		}

		if b != packetBeginLow {
			continue
		}

		second, err := reader.ReadByte()
		if err != nil {
			return err
		}
		if second != packetBeginHigh {
			continue
		}

		prefix, err := reader.ReadByte()
		if err != nil {
			return err
		}

		switch prefix {
		case packetIdle:
			payload, err := readExactly(idlePayloadSize)
			if err != nil {
				return err
			}

			errCode := payload[0]
			versionRaw := payload[1:]
			if idx := bytes.IndexByte(versionRaw, 0); idx >= 0 {
				versionRaw = versionRaw[:idx]
			}
			version := string(versionRaw)
			appendLogf("idle packet: err=0x%02X version=%s", errCode, version)
			updateStats(func(s *PacketStats) { s.IdlePackets++ })

		case packetIMU:
			payload, err := readExactly(imuPayloadSize)
			if err != nil {
				return err
			}

			want := binary.LittleEndian.Uint16(payload[:2])
			body := payload[2:]
			have := checksumCalculate(body)
			if have != want {
				appendLogf("imu packet dropped: checksum mismatch want=0x%04X got=0x%04X", want, have)
				updateStats(func(s *PacketStats) { s.ChecksumDrops++ })
				return nil
			}

			var state imuStateWire
			if err := binary.Read(bytes.NewReader(body), binary.LittleEndian, &state); err != nil {
				appendLogf("imu packet dropped: decode failed: %v", err)
				return nil
			}

			appendIMU(state)
			updateStats(func(s *PacketStats) { s.ImuPackets++ })

		default:
			appendLogf("unknown packet prefix: 0x%02X", prefix)
			updateStats(func(s *PacketStats) { s.UnknownPacket++ })
		}

		return nil
	}
}

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
	satData.RefreshRate = make([]float32, 0, 8_000_000)
	satData.CountF = make([]float32, 0, 8_000_000)
	satData.Count = make([]uint32, 0, 8_000_000)

	if binary.Size(imuStateWire{}) != 60 {
		setLogError(fmt.Errorf("imu wire struct size mismatch: got %d want 60", binary.Size(imuStateWire{})))
		appendLogf("imu wire struct size mismatch: got %d want 60", binary.Size(imuStateWire{}))
		return
	}

	setInited(false)
	setActivePort("")
	setLogError(nil)

	connect := func() {
		candidates := candidatePorts()
		appendLogf("trying to connect on ports: %v", candidates)

		var lastErr error
		for _, name := range candidates {
			config := &serial.Config{
				Name:        name,
				Baud:        115200,
				ReadTimeout: 0,
			}

			port, err := serial.OpenPort(config)
			if err != nil {
				lastErr = err
				continue
			}

			writer = bufio.NewWriter(port)
			reader = bufio.NewReader(port)

			setLogError(nil)
			setActivePort(name)
			setInited(true)
			appendLogf("serial connected on %s", name)
			updateStats(func(s *PacketStats) { s.Reconnects++ })
			return
		}

		if lastErr == nil {
			lastErr = fmt.Errorf("no serial ports available from candidate list")
		}
		setLogError(lastErr)
		time.Sleep(time.Second * 2)
	}

	go func() {
		for {
			for !getInited() {
				connect()
				continue
			}

			err := readAndParsePacket()
			if err == nil {
				continue
			}

			if err == io.EOF || err == io.ErrClosedPipe {
				appendLogf("serial disconnected from %s", getActivePort())
				setInited(false)
				setActivePort("")
				continue
			}

			setLogError(err)
			appendLogf("serial read error: %v", err)
			setInited(false)
			setActivePort("")
		}
	}()
}

func byteifyStruct(v any) ([]byte, error) {
	if v == nil {
		return nil, nil
	}

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
	sendMutex.Lock()
	defer sendMutex.Unlock()

	if err := getLogError(); err != nil {
		return err
	}

	if !getInited() {
		return fmt.Errorf("serial port not connected")
	}

	bytes, err := byteifyStruct(data)
	if err != nil {
		setLogError(err)
		return err
	}

	sendOnce := func() error {
		if err := writer.WriteByte(prefix); err != nil {
			return err
		}
		if len(bytes) > 0 {
			if _, err := writer.Write(bytes); err != nil {
				return err
			}
		}
		return writer.Flush()
	}

	attempts := 1
	if prefix == 0xBA || prefix == 0xBC || prefix == 0xBD {
		attempts = 5
	}

	for i := 0; i < attempts; i++ {
		err = sendOnce()
		if err != nil {
			setLogError(err)
			return err
		}

		if i < attempts-1 {
			time.Sleep(15 * time.Millisecond)
		}
	}

	appendLogf("sent command: 0x%02X", prefix)
	updateStats(func(s *PacketStats) { s.CommandsSent++ })
	return nil
}

func GetLogs() []string {
	logMutex.Lock()
	defer logMutex.Unlock()

	if !getInited() {
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
	return getLogError()
}

func GetData() Data {
	dataMutex.RLock()
	defer dataMutex.RUnlock()

	return satData
}

func GetDataSnapshot() *DataSnapshot {
	dataMutex.RLock()
	defer dataMutex.RUnlock()

	if len(satData.Count) == 0 {
		return nil
	}

	index := uint32(len(satData.Count) - 1)
	return &DataSnapshot{
		AccelX:      satData.AccelX[index],
		AccelY:      satData.AccelY[index],
		AccelZ:      satData.AccelZ[index],
		GyroX:       satData.GyroX[index],
		GyroY:       satData.GyroY[index],
		GyroZ:       satData.GyroZ[index],
		MagX:        satData.MagX[index],
		MagY:        satData.MagY[index],
		MagZ:        satData.MagZ[index],
		Yaw:         satData.Yaw[index],
		Pitch:       satData.Pitch[index],
		Roll:        satData.Roll[index],
		Temperature: satData.Temperature[index],
		RefreshRate: satData.RefreshRate[index],
		Count:       satData.Count[index],
		Index:       index,
	}
}

func GetStats() PacketStats {
	statsMutex.RLock()
	defer statsMutex.RUnlock()
	return satStats
}

func GetActivePort() string {
	return getActivePort()
}
