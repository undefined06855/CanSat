package can_io

import (
	"bufio"
	"bytes"
	"encoding/binary"
	"strings"
	"testing"
)

func resetTestState() {
	satData = Data{}
	satStats = PacketStats{}
	logBuffer = nil
	logError = nil
}

func TestReadAndParseIMUPacketLittleEndian(t *testing.T) {
	resetTestState()

	payloadState := imuStateWire{
		AccelX:      1.25,
		AccelY:      -2.5,
		AccelZ:      3.75,
		GyroX:       4.25,
		GyroY:       -5.5,
		GyroZ:       6.75,
		MagX:        7.25,
		MagY:        -8.5,
		MagZ:        9.75,
		Yaw:         10.25,
		Pitch:       -11.5,
		Roll:        12.75,
		Temperature: 27.5,
		Count:       123456,
		RefreshRate: 42.0,
	}

	body := new(bytes.Buffer)
	if err := binary.Write(body, binary.LittleEndian, payloadState); err != nil {
		t.Fatalf("binary.Write body failed: %v", err)
	}
	if got := body.Len(); got != 60 {
		t.Fatalf("imu body size mismatch: got %d want 60", got)
	}

	checksum := checksumCalculate(body.Bytes())
	packet := new(bytes.Buffer)
	packet.WriteByte(packetBeginLow)
	packet.WriteByte(packetBeginHigh)
	packet.WriteByte(packetIMU)
	if err := binary.Write(packet, binary.LittleEndian, checksum); err != nil {
		t.Fatalf("binary.Write checksum failed: %v", err)
	}
	packet.Write(body.Bytes())

	reader = bufio.NewReader(bytes.NewReader(packet.Bytes()))
	if err := readAndParsePacket(); err != nil {
		t.Fatalf("readAndParsePacket failed: %v", err)
	}

	if len(satData.Count) != 1 {
		t.Fatalf("expected 1 parsed sample, got %d", len(satData.Count))
	}
	if satData.Count[0] != payloadState.Count {
		t.Fatalf("count mismatch: got %d want %d", satData.Count[0], payloadState.Count)
	}
	if satData.RefreshRate[0] != payloadState.RefreshRate {
		t.Fatalf("refresh mismatch: got %v want %v", satData.RefreshRate[0], payloadState.RefreshRate)
	}
	if satStats.ImuPackets != 1 {
		t.Fatalf("imu packet stats mismatch: got %d want 1", satStats.ImuPackets)
	}
}

func TestReadAndParseIdlePacket(t *testing.T) {
	resetTestState()

	packet := []byte{
		packetBeginLow, packetBeginHigh, packetIdle,
		0xFF,
		'K', 'E', 'S', 'T', 'R', 'E', 'L', 0x00,
	}

	reader = bufio.NewReader(bytes.NewReader(packet))
	if err := readAndParsePacket(); err != nil {
		t.Fatalf("readAndParsePacket failed: %v", err)
	}

	if satStats.IdlePackets != 1 {
		t.Fatalf("idle packet stats mismatch: got %d want 1", satStats.IdlePackets)
	}
	if len(logBuffer) == 0 {
		t.Fatalf("expected idle packet log entry")
	}
}

func TestChecksumMismatchIncrementsDropCounter(t *testing.T) {
	resetTestState()

	payload := make([]byte, imuPayloadSize)
	packet := new(bytes.Buffer)
	packet.WriteByte(packetBeginLow)
	packet.WriteByte(packetBeginHigh)
	packet.WriteByte(packetIMU)
	if err := binary.Write(packet, binary.LittleEndian, uint16(0x1234)); err != nil {
		t.Fatalf("binary.Write checksum failed: %v", err)
	}
	packet.Write(payload[2:])

	reader = bufio.NewReader(bytes.NewReader(packet.Bytes()))
	if err := readAndParsePacket(); err != nil {
		t.Fatalf("readAndParsePacket failed: %v", err)
	}

	if satStats.ChecksumDrops != 1 {
		t.Fatalf("checksum drop stats mismatch: got %d want 1", satStats.ChecksumDrops)
	}
}

func TestSendDataFramesCommand(t *testing.T) {
	resetTestState()

	var out bytes.Buffer
	writer = bufio.NewWriter(&out)
	setInited(true)
	setLogError(nil)

	err := SendData(0xBA, struct{}{})
	if err != nil {
		t.Fatalf("SendData failed: %v", err)
	}

	sent := out.Bytes()
	if len(sent) < 1 {
		t.Fatalf("sent too few bytes: %d", len(sent))
	}

	if len(sent) != 5 {
		t.Fatalf("launch command byte count mismatch: got %d want 5", len(sent))
	}

	for i := range sent {
		if sent[i] != 0xBA {
			t.Fatalf("unexpected command byte at %d: 0x%02X", i, sent[i])
		}
	}
}

func TestByteifyStructNullTerminatesString(t *testing.T) {
	b, err := byteifyStruct(struct{ string }{"WR 433900 1 9 1 0"})
	if err != nil {
		t.Fatalf("byteifyStruct failed: %v", err)
	}

	if !strings.HasSuffix(string(b), "\x00") {
		t.Fatalf("expected null-terminated config string")
	}
}
