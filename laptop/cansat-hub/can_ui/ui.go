package can_ui

import (
	"bytes"
	_ "embed"
	"fmt"
	"image"
	"math"

	"undefined06855/CanSatHub/can_io"

	"github.com/AllenDang/cimgui-go/backend"
	"github.com/AllenDang/cimgui-go/backend/glfwbackend"
	"github.com/AllenDang/cimgui-go/imgui"
	"github.com/AllenDang/cimgui-go/imguizmo"
	_ "github.com/AllenDang/cimgui-go/impl/glfw"
	"github.com/AllenDang/cimgui-go/implot"
	"github.com/AllenDang/cimgui-go/utils"
)

var back backend.Backend[glfwbackend.GLFWWindowFlags]
var texID imgui.TextureID
var texRef *imgui.TextureRef
var radioConfigCommand = "WR 433900 1 9 1 0"
var gyroYawDeg float32
var gyroPitchDeg float32
var gyroRollDeg float32
var lastGyroSampleCount uint32
var lastGyroSampleIndex uint32
var hasGyroState bool

func resetGyroOrientation() {
	gyroYawDeg = 0
	gyroPitchDeg = 0
	gyroRollDeg = 0
	lastGyroSampleCount = 0
	lastGyroSampleIndex = 0
	hasGyroState = false
}

const plotWindowSamples = 300

func fixedWindow(values []float32, n int) []float32 {
	out := make([]float32, n)
	for i := range out {
		out[i] = float32(math.NaN())
	}

	if len(values) == 0 {
		return out
	}

	if len(values) >= n {
		copy(out, values[len(values)-n:])
		return out
	}

	copy(out[n-len(values):], values)
	return out
}

func integrateGyro(snapshot *can_io.DataSnapshot) (float32, float32, float32) {
	if snapshot == nil {
		return gyroYawDeg, gyroPitchDeg, gyroRollDeg
	}

	if !hasGyroState {
		hasGyroState = true
		lastGyroSampleCount = snapshot.Count
		lastGyroSampleIndex = snapshot.Index
		return gyroYawDeg, gyroPitchDeg, gyroRollDeg
	}

	if snapshot.Index == lastGyroSampleIndex {
		return gyroYawDeg, gyroPitchDeg, gyroRollDeg
	}

	deltaMs := int64(snapshot.Count) - int64(lastGyroSampleCount)
	dt := float32(0)
	if deltaMs > 0 {
		dt = float32(deltaMs) / 1000.0
	}
	if (dt <= 0 || dt > 1.0) && snapshot.RefreshRate > 0 {
		dt = 1.0 / snapshot.RefreshRate
	}
	if dt > 0.25 {
		dt = 0.25
	}

	gyroYawDeg += snapshot.GyroZ * dt
	gyroPitchDeg += snapshot.GyroY * dt
	gyroRollDeg += snapshot.GyroX * dt

	lastGyroSampleCount = snapshot.Count
	lastGyroSampleIndex = snapshot.Index

	return gyroYawDeg, gyroPitchDeg, gyroRollDeg
}

//go:embed no-gyro.png
var imgData []byte
var placeholderTexture *backend.Texture

func AfterCreateContext() {
	Setup3DScene()
	texID = imgui.TextureID(fboTex)
	texRef = imgui.NewTextureRefNil()
	texRef.SetTexID(texID)

	implot.CreateContext()

	imgImage, _, _ := image.Decode(bytes.NewReader(imgData))
	img := backend.ImageToRgba(imgImage)
	placeholderTexture = backend.NewTextureFromRgba(img)
}

func BeforeDestroyContext() {
	implot.DestroyContext()
}

func SetWindowTitle(title string) {
	back.SetWindowTitle(fmt.Sprintf("CanSat - %s", title))
}

func Setup(_back backend.Backend[glfwbackend.GLFWWindowFlags]) {
	back = _back

	back.SetAfterCreateContextHook(AfterCreateContext)
	back.SetBeforeDestroyContextHook(BeforeDestroyContext)

	back.SetBgColor(imgui.NewVec4(0.45, 0.55, 0.6, 1.0))

	back.CreateWindow("...", 1920, 1080)
	SetWindowTitle("Idle")

	// font and other styles
	imgui.PushFont(imgui.CurrentIO().Fonts().AddFontFromFileTTF("resources/DMMono.ttf"), 32)

	// prevent our window popping out
	config := imgui.CurrentIO().ConfigFlags()
	config &^= imgui.ConfigFlagsViewportsEnable
	imgui.CurrentIO().SetConfigFlags(config)

	back.Run(Loop)
}

func Loop() {
	vp := imgui.MainViewport()

	imgui.SetNextWindowPos(vp.Pos())
	imgui.SetNextWindowSize(vp.Size())

	imgui.BeginV("root", nil,
		imgui.WindowFlagsNoTitleBar|
			imgui.WindowFlagsNoResize|
			imgui.WindowFlagsNoMove)

	if imgui.BeginTabBar("tabs") {
		if imgui.BeginTabItem("Logs") {
			Logs()
			imgui.EndTabItem()
		}

		if imgui.BeginTabItem("Actions") {
			Actions()
			imgui.EndTabItem()
		}

		if imgui.BeginTabItem("Graphs") {
			Graphs()
			imgui.EndTabItem()
		}

		imgui.EndTabBar()
	}

	imgui.End()
}

func Logs() {
	stats := can_io.GetStats()
	port := can_io.GetActivePort()
	if port == "" {
		port = "not connected"
	}

	imgui.Text(fmt.Sprintf(
		"port=%s | idle=%d imu=%d checksumDrops=%d unknown=%d sent=%d reconnects=%d",
		port,
		stats.IdlePackets,
		stats.ImuPackets,
		stats.ChecksumDrops,
		stats.UnknownPacket,
		stats.CommandsSent,
		stats.Reconnects,
	))
	imgui.Separator()

	if can_io.GetError() != nil {
		imgui.PushStyleColorVec4(imgui.ColText, imgui.NewVec4(1.0, 0.0, 0.0, 1.0))
	}

	start := max(0, len(can_io.GetLogs())-25) // 25 lines at a time
	for _, line := range can_io.GetLogs()[start:] {
		imgui.Text(line)
	}

	if can_io.GetError() != nil {
		imgui.PopStyleColor()
	}
}

func Actions() {
	imgui.Text("actions")

	imgui.InputTextWithHint(" ", "WR 433900 1 9 1 0", &radioConfigCommand, imgui.InputTextFlagsNone, nil)
	imgui.SameLine()
	if imgui.Button("Send Radio Config 0xBB") {
		err := can_io.SendData(
			0xBB,
			struct {
				string
			}{
				radioConfigCommand,
			},
		)
		if err != nil {
			fmt.Printf("failed sending radio config: %v\n", err)
		}
	}

	imgui.Separator()

	imgui.PushStyleVarVec2(imgui.StyleVarFramePadding, imgui.NewVec2(100, 100))
	if imgui.Button("LAUNCH 0xBA") {
		err := can_io.SendData(
			0xBA,
			struct {
			}{},
		)
		if err != nil {
			fmt.Printf("failed sending launch packet: %v\n", err)
		}
		SetWindowTitle("Launched")
	}
	imgui.PopStyleVar()

	imgui.SameLine()

	imgui.PushStyleVarVec2(imgui.StyleVarFramePadding, imgui.NewVec2(40, 40))
	if imgui.Button("land 0xBC") {
		err := can_io.SendData(
			0xBC,
			struct {
			}{},
		)
		if err != nil {
			fmt.Printf("failed sending land packet: %v\n", err)
		}
		SetWindowTitle("Landed. Well done!")
	}
	imgui.PopStyleVar()

	imgui.PushStyleVarVec2(imgui.StyleVarFramePadding, imgui.NewVec2(20, 20))
	if imgui.Button("beep beep beep beep 0xBD") {
		err := can_io.SendData(
			0xBD,
			struct {
			}{},
		)
		if err != nil {
			fmt.Printf("failed sending buzzer packet: %v\n", err)
		}
	}
	imgui.PopStyleVar()
}

func Graphs() {
	imgui.BeginChildStrV("graphs_panel", imgui.NewVec2(imgui.ContentRegionAvail().X*0.6, 0), imgui.ChildFlagsNone, imgui.WindowFlagsNone)

	Plots()

	imgui.EndChild()

	imgui.SameLine()

	imgui.BeginChildStrV("3d_panel", imgui.NewVec2(0, 0), imgui.ChildFlagsNone, imgui.WindowFlagsNone)

	imguizmo.BeginFrame()
	if imgui.Button("Reset Angle") {
		resetGyroOrientation()
	}
	imgui.SameLine()
	imgui.Text(fmt.Sprintf("Y: %.1f  P: %.1f  R: %.1f", gyroYawDeg, gyroPitchDeg, gyroRollDeg))

	texSize := imgui.ContentRegionAvail()
	snapshot := can_io.GetDataSnapshot()
	if snapshot != nil {
		yawDeg, pitchDeg, rollDeg := integrateGyro(snapshot)
		Render3DSceneWithGyro(yawDeg, pitchDeg, rollDeg)
		imgui.ImageV(*texRef, imgui.NewVec2(texSize.X, texSize.X), imgui.NewVec2(0, 1), imgui.NewVec2(1, 0))
	} else {
		hasGyroState = false
		imgui.Image(placeholderTexture.ID, imgui.NewVec2(float32(placeholderTexture.Width), float32(placeholderTexture.Height)))
	}

	imgui.EndChild()
}

func Plots() {
	data := can_io.GetData()
	snapshot := can_io.GetDataSnapshot()

	avail := imgui.ContentRegionAvail()
	if snapshot != nil {
		imgui.Text(fmt.Sprintf("Count: %d ms | Samples: %d", snapshot.Count, snapshot.Index+1))
	} else {
		imgui.Text("Count: n/a | Samples: 0")
	}
	imgui.Separator()

	plotAreaHeight := imgui.ContentRegionAvail().Y
	plotSize := imgui.NewVec2(avail.X*0.5-6, plotAreaHeight/3.0-10)

	plotXYZ := func(title, yLabel string, x, y, z []float32, fixedY bool, yMin, yMax float64) {
		if implot.BeginPlotV(title, plotSize, implot.FlagsNone) {
			implot.SetupAxes("Samples", yLabel)
			implot.SetupAxisLimitsConstraints(implot.AxisX1, 0, plotWindowSamples-1)
			if fixedY {
				implot.SetupAxesLimits(0, plotWindowSamples-1, yMin, yMax)
			}

			x = fixedWindow(x, plotWindowSamples)
			y = fixedWindow(y, plotWindowSamples)
			z = fixedWindow(z, plotWindowSamples)

			if len(x) > 0 {
				implot.PlotLineFloatPtrInt("X", utils.SliceToPtr(x), int32(len(x)))
			}
			if len(y) > 0 {
				implot.PlotLineFloatPtrInt("Y", utils.SliceToPtr(y), int32(len(y)))
			}
			if len(z) > 0 {
				implot.PlotLineFloatPtrInt("Z", utils.SliceToPtr(z), int32(len(z)))
			}

			implot.EndPlot()
		}
	}

	plotSingle := func(title, yLabel, lineLabel string, values []float32) {
		if implot.BeginPlotV(title, plotSize, implot.FlagsNone) {
			implot.SetupAxes("Samples", yLabel)
			implot.SetupAxisLimitsConstraints(implot.AxisX1, 0, plotWindowSamples-1)
			values = fixedWindow(values, plotWindowSamples)

			if len(values) > 0 {
				implot.PlotLineFloatPtrInt(lineLabel, utils.SliceToPtr(values), int32(len(values)))
			}

			implot.EndPlot()
		}
	}

	plotXYZ("Accelerometer", "mG", data.AccelX, data.AccelY, data.AccelZ, false, 0, 0)
	imgui.SameLine()
	plotXYZ("Gyroscope", "deg/s", data.GyroX, data.GyroY, data.GyroZ, true, -200, 200)

	plotXYZ("Magnetometer", "raw", data.MagX, data.MagY, data.MagZ, false, 0, 0)
	imgui.SameLine()
	plotXYZ("Orientation", "deg", data.Yaw, data.Pitch, data.Roll, false, 0, 0)

	plotSingle("Temperature", "C", "Temperature", data.Temperature)
	imgui.SameLine()
	plotSingle("IMU Refresh Rate", "Hz", "Refresh", data.RefreshRate)
}
