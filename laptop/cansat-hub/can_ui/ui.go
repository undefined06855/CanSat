package can_ui

import (
    "fmt"
    "bytes"
    "image"
    _ "embed"

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

//go:embed no-gyro.png
var	imgData []byte
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
    imgui.PushFont(imgui.CurrentIO().Fonts().AddFontFromFileTTF("resources/DMMono.ttf"), 32);

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
        imgui.WindowFlagsNoTitleBar |
        imgui.WindowFlagsNoResize |
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
    if can_io.GetError() != nil {
        imgui.PushStyleColorVec4(imgui.ColText, imgui.NewVec4(1.0, 0.0, 0.0, 1.0))
    }

    start := max(0, len(can_io.GetLogs()) - 25) // 25 lines at a time
    for _, line := range can_io.GetLogs()[start:] {
        imgui.Text(line)
    }

    if can_io.GetError() != nil {
        imgui.PopStyleColor()
    }
}

func Actions() {
    imgui.Text("actions")

    var buf string
    imgui.InputTextWithHint(" ", "WR 433900 1 9 1 0", &buf, imgui.InputTextFlagsNone, nil)
    imgui.SameLine()
    if imgui.Button("Send Radio Config 0xBB") {
        can_io.SendData(
            0xBB,
            struct {
                string
            }{
                buf,
            },
        )

        fmt.Printf("sent radio config %v!\n", buf)
    }

    imgui.Separator()

    imgui.PushStyleVarVec2(imgui.StyleVarFramePadding, imgui.NewVec2(100, 100))
    if imgui.Button("LAUNCH 0xBA") {
        can_io.SendData(
            0xBA,
            struct {
            }{
            },
        )

        fmt.Printf("sent launch packet %v!\n", buf)
        SetWindowTitle("Launched")
    }
    imgui.PopStyleVar()

    imgui.SameLine()

    imgui.PushStyleVarVec2(imgui.StyleVarFramePadding, imgui.NewVec2(40, 40))
    if imgui.Button("land 0xBC") {
        can_io.SendData(
            0xBC,
            struct {
            }{
            },
        )

        fmt.Printf("sent land packet %v!\n", buf)
        SetWindowTitle("Landed. Well done!")
    }
    imgui.PopStyleVar()

    imgui.PushStyleVarVec2(imgui.StyleVarFramePadding, imgui.NewVec2(20, 20))
    if imgui.Button("beep beep beep beep 0xBD") {
        can_io.SendData(
            0xBD,
            struct {
            }{
            },
        )

        fmt.Printf("testing buzzer %v!\n", buf)
    }
    imgui.PopStyleVar()
}

func Graphs() {
    imgui.BeginChildStrV("graphs_panel", imgui.NewVec2(imgui.ContentRegionAvail().X * 0.6, 0), imgui.ChildFlagsNone, imgui.WindowFlagsNone)

    Plots()

    imgui.EndChild()

    imgui.SameLine()

    imgui.BeginChildStrV("3d_panel", imgui.NewVec2(0, 0), imgui.ChildFlagsNone, imgui.WindowFlagsNone)

    imguizmo.BeginFrame()

    texSize := imgui.ContentRegionAvail()
    snapshot := can_io.GetDataSnapshot()
    if snapshot != nil {
        Render3DSceneWithGyro(snapshot.GyroX, snapshot.GyroY, snapshot.GyroZ)
        imgui.ImageV(*texRef, imgui.NewVec2(texSize.X, texSize.X), imgui.NewVec2(0, 1), imgui.NewVec2(1, 0))
    } else {
        imgui.Image(placeholderTexture.ID, imgui.NewVec2(float32(placeholderTexture.Width), float32(placeholderTexture.Height)))
    }


    imgui.EndChild()
}

func Plots() {
    data := can_io.GetData()

    avail := imgui.ContentRegionAvail()

    width := 120.0

    if implot.BeginPlotV("Pressure", imgui.NewVec2(avail.X * 0.5, avail.Y * 0.5), implot.FlagsNone) {
        implot.SetupAxes("Time (s)", "Pressure (mPa)")
        implot.SetupAxesLimits(0, width, 300, 1100)
        implot.SetupAxisLimitsConstraints(implot.AxisX1, 0, width)
        implot.SetupAxisLimitsConstraints(implot.AxisY1, 300, 1100)

        implot.PlotLineFloatPtrInt(
            "Line",
            utils.SliceToPtr(data.Pressure),
            int32(len(data.Pressure)),
        )

        implot.EndPlot()
    }

    imgui.SameLine()

    if implot.BeginPlotV("Temperature", imgui.NewVec2(avail.X * 0.5, avail.Y * 0.5), implot.FlagsNone) {
        implot.SetupAxes("Time (s)", "Temperature (C)")
        implot.SetupAxesLimits(0, width, -20, 30)
        implot.SetupAxisLimitsConstraints(implot.AxisX1, 0, width)
        implot.SetupAxisLimitsConstraints(implot.AxisY1, -20, 30)

        implot.PlotLineFloatPtrInt(
            "Line",
            utils.SliceToPtr(data.Temperature),
            int32(len(data.Temperature)),
        )

        implot.EndPlot()
    }
}
