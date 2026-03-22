package can_ui

// so like... im not saying this is ALL chatgpt but

import (
	"log"
	"runtime"
	"unsafe"

	"github.com/go-gl/gl/v4.6-core/gl"
	"github.com/go-gl/mathgl/mgl32"
	"github.com/udhos/gwob"
)

var (
    // FBO & texture
    fbo    uint32
    fboTex uint32

    // mesh
    meshVAO     uint32
    meshVBO     uint32
    meshEBO     uint32
    meshIndCount int32

    // shader program
    prog uint32

    // rotation
    angle float32
)

const (
    renderWidth  = 512
    renderHeight = 512
)

// Setup3DScene loads OBJ, sets up buffers, shaders, framebuffer.
// Returns the OpenGL texture handle usable in ImGui.
func Setup3DScene() uint32 {
    runtime.LockOSThread()

    if gl.Init() != nil {
        log.Fatalln("gl.Init failed")
    }

    // Load OBJ
    obj, err := gwob.NewObjFromFile("resources/hull2.obj", &gwob.ObjParserOptions{})
    if err != nil {
        log.Fatalln("failed loading OBJ:", err)
    }

    // Prepare indices
    inds := make([]uint32, len(obj.Indices))
    for i, v := range obj.Indices {
        inds[i] = uint32(v)
    }
    meshIndCount = int32(len(inds))

    // VAO/VBO/EBO
    gl.GenVertexArrays(1, &meshVAO)
    gl.GenBuffers(1, &meshVBO)
    gl.GenBuffers(1, &meshEBO)

    gl.BindVertexArray(meshVAO)

    gl.BindBuffer(gl.ARRAY_BUFFER, meshVBO)
    gl.BufferData(gl.ARRAY_BUFFER, len(obj.Coord)*4, gl.Ptr(obj.Coord), gl.STATIC_DRAW)

    gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, meshEBO)
    gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(inds)*4, gl.Ptr(inds), gl.STATIC_DRAW)

    // Attributes
    stride := int32(obj.StrideSize)
    gl.EnableVertexAttribArray(0) // position
    gl.VertexAttribPointer(0, 3, gl.FLOAT, false, stride, unsafe.Pointer(uintptr(obj.StrideOffsetPosition)))
    if obj.NormCoordFound {
        gl.EnableVertexAttribArray(1) // normal
        gl.VertexAttribPointer(1, 3, gl.FLOAT, false, stride, unsafe.Pointer(uintptr(obj.StrideOffsetNormal)))
    }

    gl.BindVertexArray(0)

    // Shader
    prog = createSimpleShader()

    // Framebuffer + texture
    gl.GenFramebuffers(1, &fbo)
    gl.BindFramebuffer(gl.FRAMEBUFFER, fbo)

    gl.GenTextures(1, &fboTex)
    gl.BindTexture(gl.TEXTURE_2D, fboTex)
    gl.TexImage2D(gl.TEXTURE_2D, 0, gl.RGBA, renderWidth, renderHeight, 0, gl.RGBA, gl.UNSIGNED_BYTE, nil)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
    gl.FramebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, fboTex, 0)

    // Depth
    var rbo uint32
    gl.GenRenderbuffers(1, &rbo)
    gl.BindRenderbuffer(gl.RENDERBUFFER, rbo)
    gl.RenderbufferStorage(gl.RENDERBUFFER, gl.DEPTH24_STENCIL8, renderWidth, renderHeight)
    gl.FramebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_STENCIL_ATTACHMENT, gl.RENDERBUFFER, rbo)

    if gl.CheckFramebufferStatus(gl.FRAMEBUFFER) != gl.FRAMEBUFFER_COMPLETE {
        log.Fatalln("framebuffer not complete")
    }
    gl.BindFramebuffer(gl.FRAMEBUFFER, 0)

    return fboTex
}

// Render3DSceneWithGyro renders the mesh with a rotation given by yaw, pitch, roll in degrees.
// Yaw = rotation around global Y axis
// Pitch = rotation around local X axis
// Roll = rotation around local Z axis (axis pointing "up" in the IMU)
func Render3DSceneWithGyro(yawDeg, pitchDeg, rollDeg float32) {
    gl.BindFramebuffer(gl.FRAMEBUFFER, fbo)
    gl.Viewport(0, 0, renderWidth, renderHeight)
    gl.ClearColor(0.1, 0.1, 0.1, 1)
    gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)
    gl.Enable(gl.DEPTH_TEST)

    gl.UseProgram(prog)

    // ------------------ Transform matrices ------------------
    proj := mgl32.Perspective(mgl32.DegToRad(45), float32(renderWidth)/renderHeight, 0.1, 100)
    view := mgl32.LookAtV(
        mgl32.Vec3{15.66, 13.994, -20.73},
        mgl32.Vec3{0, 0, 0},
        mgl32.Vec3{0, 1, 0},
    )

    // Convert degrees to radians
    yaw := mgl32.DegToRad(yawDeg)
    pitch := mgl32.DegToRad(pitchDeg)
    roll := mgl32.DegToRad(rollDeg)

    // Build rotation matrices
    rotYaw := mgl32.HomogRotate3DY(yaw)     // rotate around Y (up)
    rotPitch := mgl32.HomogRotate3DX(pitch) // rotate around X
    rotRoll := mgl32.HomogRotate3DZ(roll)   // rotate around Z (IMU roll axis up)

    // Combine rotations: roll -> pitch -> yaw
    model := rotYaw.Mul4(rotPitch).Mul4(rotRoll)

    mvp := proj.Mul4(view).Mul4(model)

    // Send MVP
    locMVP := gl.GetUniformLocation(prog, gl.Str("MVP\x00"))
    gl.UniformMatrix4fv(locMVP, 1, false, &mvp[0])

    // Send model matrix
    locModel := gl.GetUniformLocation(prog, gl.Str("model\x00"))
    gl.UniformMatrix4fv(locModel, 1, false, &model[0])

    // Normal matrix
    normalMat := model.Mat3().Inv().Transpose()
    locNormal := gl.GetUniformLocation(prog, gl.Str("normalMatrix\x00"))
    gl.UniformMatrix3fv(locNormal, 1, false, &normalMat[0])

    // ------------------ Lighting ------------------
    lightDir := mgl32.Vec3{-0.5, -1.0, -0.3}.Normalize()
    locLightDir := gl.GetUniformLocation(prog, gl.Str("lightDir\x00"))
    gl.Uniform3fv(locLightDir, 1, &lightDir[0])

    lightColor := mgl32.Vec3{1, 1, 1}
    locLightColor := gl.GetUniformLocation(prog, gl.Str("lightColor\x00"))
    gl.Uniform3fv(locLightColor, 1, &lightColor[0])

    ambientColor := mgl32.Vec3{0.2, 0.2, 0.2}
    locAmbient := gl.GetUniformLocation(prog, gl.Str("ambientColor\x00"))
    gl.Uniform3fv(locAmbient, 1, &ambientColor[0])

    objectColor := mgl32.Vec3{0.7, 0.8, 1.0} // base color
    locObj := gl.GetUniformLocation(prog, gl.Str("objectColor\x00"))
    gl.Uniform3fv(locObj, 1, &objectColor[0])

    // ------------------ Draw ------------------
    gl.BindVertexArray(meshVAO)
    gl.DrawElements(gl.TRIANGLES, meshIndCount, gl.UNSIGNED_INT, unsafe.Pointer(nil))
    gl.BindVertexArray(0)

    gl.BindFramebuffer(gl.FRAMEBUFFER, 0)
}

// -------------------- shader helpers --------------------

func createSimpleShader() uint32 {
    vertexShader := `
#version 330 core
layout(location=0) in vec3 pos;
layout(location=1) in vec3 nor;

uniform mat4 MVP;
uniform mat4 model; // For transforming normals to world space
uniform mat3 normalMatrix; // Usually inverse transpose of model's upper 3x3

out vec3 FragPos;
out vec3 Normal;

void main() {
    FragPos = vec3(model * vec4(pos, 1.0));
    Normal = normalize(normalMatrix * nor);
    gl_Position = MVP * vec4(pos, 1.0);
}
    ` + "\x00"

    fragmentShader := `
#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightDir;    // normalized direction of the light
uniform vec3 lightColor;  // e.g., vec3(1.0, 1.0, 1.0)
uniform vec3 ambientColor; // e.g., vec3(0.2, 0.2, 0.2)
uniform vec3 objectColor;  // base color of your object

void main() {
    // Diffuse shading (Lambert)
    float diff = max(dot(normalize(Normal), normalize(-lightDir)), 0.0);
    vec3 diffuse = diff * lightColor;

    // Ambient shading
    vec3 ambient = ambientColor;

    vec3 result = (ambient + diffuse) * objectColor;
    FragColor = vec4(result, 1.0);
}
    ` + "\x00"

    return compileProgram(vertexShader, fragmentShader)
}

func compileProgram(vsSource, fsSource string) uint32 {
    vs := gl.CreateShader(gl.VERTEX_SHADER)
    csources, free := gl.Strs(vsSource)
    gl.ShaderSource(vs, 1, csources, nil)
    free()
    gl.CompileShader(vs)
    checkShaderCompile(vs)

    fs := gl.CreateShader(gl.FRAGMENT_SHADER)
    csources, free = gl.Strs(fsSource)
    gl.ShaderSource(fs, 1, csources, nil)
    free()
    gl.CompileShader(fs)
    checkShaderCompile(fs)

    prog := gl.CreateProgram()
    gl.AttachShader(prog, vs)
    gl.AttachShader(prog, fs)
    gl.LinkProgram(prog)
    checkProgramLink(prog)

    gl.DeleteShader(vs)
    gl.DeleteShader(fs)
    return prog
}
func checkShaderCompile(shader uint32) {
    var status int32
    gl.GetShaderiv(shader, gl.COMPILE_STATUS, &status)

    var logLen int32
    gl.GetShaderiv(shader, gl.INFO_LOG_LENGTH, &logLen)

    if logLen > 0 {
        infoLog := make([]byte, logLen+1)
        var actualLen int32
        gl.GetShaderInfoLog(shader, logLen, &actualLen, &infoLog[0])
        println("shader log:", string(infoLog[:actualLen]))
    }

    if status == gl.FALSE {
        log.Fatalln("shader compile failed")
    }
}

func checkProgramLink(prog uint32) {
    var status int32
    gl.GetProgramiv(prog, gl.LINK_STATUS, &status)

    var logLen int32
    gl.GetProgramiv(prog, gl.INFO_LOG_LENGTH, &logLen)

    if logLen > 0 {
        infoLog := make([]byte, logLen+1)
        var actualLen int32
        gl.GetProgramInfoLog(prog, logLen, &actualLen, &infoLog[0])
        println("program link log:", string(infoLog[:actualLen]))
    }

    if status == gl.FALSE {
        log.Fatalln("program link failed")
    }
}