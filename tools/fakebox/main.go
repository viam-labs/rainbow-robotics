// Command fakebox runs the in-process fake control box as a standalone server
// so the module can be exercised under a local viam-server without hardware:
//
//	go run ./tools/fakebox -cmd-port 5000 -data-port 5001
//
// then configure the arm with "host": "127.0.0.1".
package main

import (
	"flag"
	"fmt"
	"os"
	"os/signal"
	"syscall"

	"github.com/viam-labs/rainbow-robotics/rb/fakebox"
)

func main() {
	cmdPort := flag.Int("cmd-port", 5000, "command port to listen on")
	dataPort := flag.Int("data-port", 5001, "data port to listen on")
	addr := flag.String("addr", "127.0.0.1", "interface to listen on")
	activated := flag.Bool("activated", false, "start with the arm already activated")
	real := flag.Bool("real", false, "start in real mode instead of simulation")
	flag.Parse()

	box, err := fakebox.Start(fakebox.Options{
		CommandPort: *cmdPort, DataPort: *dataPort, Addr: *addr, Activated: *activated, Real: *real,
	})
	if err != nil {
		fmt.Fprintln(os.Stderr, "fakebox:", err)
		os.Exit(1)
	}
	fmt.Printf("fake control box listening on %s (command %d, data %d)\n", box.Host(), box.CommandPort(), box.DataPort())
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)
	<-sig
	box.Close()
}
