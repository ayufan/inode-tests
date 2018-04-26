package main

import (
	"bufio"
	"flag"
	"fmt"
	"io"
	"log"
	"net/http"
	"sync"

	serial "go.bug.st/serial.v1"
	"go.bug.st/serial.v1/enumerator"
	"golang.org/x/net/websocket"
)

type device struct {
	Name     string
	VID, PID string
}

var httpAddr = flag.String("http-addr", "127.0.0.1:8011", "HTTP server address")
var debug = flag.Bool("debug", false, "Print debug messages")

var globalPort serial.Port
var globalLock sync.Mutex

var devices = []device{
	device{Name: "stm", VID: "0483", PID: "5740"},
}

var baudRates = []int{
	115200,
	921600,
}

func logRequest(req *http.Request, v ...interface{}) {
	log.Print(":: ", req.RequestURI, " :: ", fmt.Sprintln(v...))
}

func openPort(usbVid, usbPid string, baudRate int) (serial.Port, *enumerator.PortDetails, error) {
	ports, err := enumerator.GetDetailedPortsList()
	if err != nil {
		return nil, nil, err
	}
	if *debug {
		log.Printf("Looking for Serial pid=%s, vid=%s...\n", usbVid, usbPid)
	}

	globalLock.Lock()
	defer globalLock.Unlock()

	if globalPort != nil {
		globalPort.Close()
		globalPort = nil
	}

	for _, portDetails := range ports {
		if !portDetails.IsUSB {
			continue
		}
		if *debug {
			log.Printf("Serial port: %s, vid=%s, pid=%s, sn=%s\n",
				portDetails.Name,
				portDetails.VID, portDetails.PID,
				portDetails.SerialNumber)
		}

		if portDetails.VID == usbVid && portDetails.PID == usbPid {
			port, err := serial.Open(portDetails.Name, &serial.Mode{
				BaudRate: baudRate,
				DataBits: 8,
				Parity:   serial.NoParity,
				StopBits: serial.OneStopBit,
			})
			if err == nil {
				globalPort = port
			}
			return port, portDetails, err
		}
	}

	return nil, nil, fmt.Errorf("no USB-Serial vid=%s, pid=%s", usbVid, usbPid)
}

func copyData(req *http.Request, w io.Writer, r io.Reader, dir string) error {
	buffered := bufio.NewReader(r)

	for {
		data, _, err := buffered.ReadLine()
		if err != nil {
			return err
		}

		n, err := w.Write(data)
		if err != nil {
			return err
		}
		if n != len(data) {
			return fmt.Errorf("failed to write %d bytes", n)
		}

		logRequest(req, dir, string(data))
	}
}

func proxySerial(ws *websocket.Conn, usbVid, usbPid string, baudRate int) {
	defer ws.Close()

	port, portDetails, err := openPort(usbVid, usbPid, baudRate)
	if err != nil {
		logRequest(ws.Request(), "!! error=", err)
		return
	}
	defer port.Close()

	logRequest(ws.Request(), "!! connection open:", portDetails.Name, "vid="+portDetails.VID, "pid="+portDetails.PID)

	go func() {
		err := copyData(ws.Request(), port, ws, ">>")
		if err != nil {
			ws.WriteClose(http.StatusServiceUnavailable)
		} else {
			ws.WriteClose(http.StatusOK)
		}
	}()

	err = copyData(ws.Request(), ws, port, "<<")
	if err != nil {
		ws.WriteClose(http.StatusServiceUnavailable)
	}

	logRequest(ws.Request(), "!! connection close:", portDetails.Name, "status=", err)
}

func handleStm921600(ws *websocket.Conn) {
	proxySerial(ws, "0483", "5740", 921600)
}

func proxySerialHandler(device device, baudRate int) websocket.Handler {
	return websocket.Handler(func(ws *websocket.Conn) {
		proxySerial(ws, device.VID, device.PID, baudRate)
	})
}

func main() {
	flag.Parse()

	println("iNode USB Server, Kamil Trzciński, 2018")
	println("")
	println("Open:")

	for _, device := range devices {
		for _, baudRate := range baudRates {
			path := fmt.Sprintf("/%s/%d", device.Name, baudRate)
			http.Handle(path, proxySerialHandler(device, baudRate))
			fmt.Printf(" - https://support.inode.pl/apps/iNodeLoraMonitor/index.html?host=%s%s\n", *httpAddr, path)
		}
	}

	println("")

	println("Starting", *httpAddr, "...")
	println("")

	err := http.ListenAndServe(*httpAddr, nil)
	if err != nil {
		log.Panic(err)
	}
}
