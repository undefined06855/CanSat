package main

import (
    "log"
    "runtime"

    "undefined06855/CanSatHub/can_ui"
    "undefined06855/CanSatHub/can_io"

    "github.com/AllenDang/cimgui-go/backend"
    "github.com/AllenDang/cimgui-go/backend/glfwbackend"
)


func init() {
    runtime.LockOSThread()
    can_io.InitLogs()
}

func main() {
    back, err := backend.CreateBackend(glfwbackend.NewGLFWBackend())

    if err != nil {
        log.Fatal(err)
    }

    can_ui.Setup(back)
}
