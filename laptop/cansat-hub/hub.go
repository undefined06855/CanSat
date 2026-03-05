package main

import (
    "log"
    "runtime"
    
    "github.com/AllenDang/cimgui-go/backend"
    "github.com/AllenDang/cimgui-go/backend/glfwbackend"
)


func init() {
    runtime.LockOSThread()
    initLogs()
}

func main() {
    back, err := backend.CreateBackend(glfwbackend.NewGLFWBackend())

    if err != nil {
        log.Fatal(err)
    }

    setup(back)
}
