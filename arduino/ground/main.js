import { SerialPort } from "serialport"
import { ReadlineParser } from "@serialport/parser-readline"

process.on("uncaughtException", console.error)
process.on("unhandledRejection", console.error)

const port = new SerialPort("/dev/ttyACM0", { baudRate: 9600 })
const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }))

port.on("open", () => {
    console.log("serial port open")

    port.write("hiii\n", err => {
        if (err) throw err
    })
})

parser.on("data", data =>{
    console.log(data)
})
