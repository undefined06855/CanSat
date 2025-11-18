import { SerialPort } from "serialport"
import { ReadlineParser } from "@serialport/parser-readline"

process.on("uncaughtException", console.error)
process.on("unhandledRejection", console.error)

// wait for uno to connect
/** @type {SerialPort<any>} */
let port = await (async () => {
    return new Promise(resolve => {
        let port

        function attempt() {
            port = new SerialPort({
                path: "/dev/ttyACM0",
                baudRate: 9600
            }, err => {
                if (err) {
                    console.clear()
                    console.log("(waiting for ground uno...)")
                    setTimeout(attempt, 300)
                    return
                }

                resolve(port)
            })
        }

        attempt()
    })
})()

console.clear()

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
