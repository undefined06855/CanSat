package main

import (
    "fmt"

    "github.com/AllenDang/cimgui-go/backend"
    "github.com/AllenDang/cimgui-go/backend/glfwbackend"
    "github.com/AllenDang/cimgui-go/imgui"
    _ "github.com/AllenDang/cimgui-go/impl/glfw"
)

var back backend.Backend[glfwbackend.GLFWWindowFlags]

func afterCreateContext() {

}

func beforeDestroyContext() {

}

func setWindowTitle(title string) {
    back.SetWindowTitle(fmt.Sprintf("CanSat - %s", title))
}

func setup(_back backend.Backend[glfwbackend.GLFWWindowFlags]) {
    back = _back

    back.SetAfterCreateContextHook(afterCreateContext)
    back.SetBeforeDestroyContextHook(beforeDestroyContext)

    back.SetBgColor(imgui.NewVec4(0.45, 0.55, 0.6, 1.0))

    back.CreateWindow("...", 1920, 1080)
    setWindowTitle("Disconnected")

    // font and other styles
    imgui.PushFont(imgui.CurrentIO().Fonts().AddFontFromFileTTF("resources/DMMono.ttf"), 32);

    // prevent our window popping out
    config := imgui.CurrentIO().ConfigFlags()
    config &^= imgui.ConfigFlagsViewportsEnable
    imgui.CurrentIO().SetConfigFlags(config)

    back.Run(loop)
}

func loop() {
    vp := imgui.MainViewport()

    imgui.SetNextWindowPos(vp.Pos())
    imgui.SetNextWindowSize(vp.Size())

    imgui.BeginV("root", nil,
        imgui.WindowFlagsNoTitleBar |
        imgui.WindowFlagsNoResize |
        imgui.WindowFlagsNoMove)

    if imgui.BeginTabBar("tabs") {
        if imgui.BeginTabItem("Logs") {
            logs()
            imgui.EndTabItem()
        }

        if imgui.BeginTabItem("Actions") {
            actions()
            imgui.EndTabItem()
        }

        if imgui.BeginTabItem("Graphs") {
            graphs()
            imgui.EndTabItem()
        }

        imgui.EndTabBar()
    }

    imgui.End()
}

func logs() {
    if logError != nil {
        imgui.PushStyleColorVec4(imgui.ColText, imgui.NewVec4(1.0, 0.0, 0.0, 1.0))
        imgui.Text("log error:")
        imgui.SameLine()
        imgui.Text(logError.Error())
        imgui.PopStyleColor()
    }

    start := max(0, len(logBuffer) - 25) // 25 lines at a time
    for _, line := range logBuffer[start:] {
        imgui.Text(line)
    }
}

func actions() {
    imgui.Text("actions")

    var buf string
    imgui.InputTextWithHint("Radio Config", "WR 433900 1 9 1 0", &buf, imgui.InputTextFlagsNone, nil)
    if imgui.Button("Send") {
        sendData(
            0xBB,
            struct {
                string
            }{
                buf,
            },
        )

        fmt.Printf("sent %v!\n", buf)
    }
}

func graphs() {
    imgui.Text("graphs")
}
