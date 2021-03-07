package roslibgo

import (
	"fmt"
	"os"
	"os/exec"
	"syscall"
	"testing"
	"time"
)

type rosCmd struct {
	roscore   *exec.Cmd
	roslaunch *exec.Cmd
}

func preprocess() *rosCmd {
	roscore := exec.Command("roscore")
	roscore.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	roscore.Start()
	time.Sleep(3000 * time.Millisecond)

	roslaunch := exec.Command("roslaunch", "rosbridge_server", "rosbridge_websocket.launch")
	roslaunch.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	roslaunch.Start()
	time.Sleep(3000 * time.Millisecond)
	return &rosCmd{roscore: roscore, roslaunch: roslaunch}
}

func (roscmd rosCmd) postprocess() {
	// kill roslaunch
	syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGINT)
	// syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGKILL)
	roscmd.roslaunch.Wait()

	// kill roscore
	syscall.Kill(-roscmd.roscore.Process.Pid, syscall.SIGINT)
	// syscall.Kill(-roscmd.roscore.Process.Pid, syscall.SIGKILL)
	roscmd.roscore.Wait()
	fmt.Println("postprocess done")
}

func TestMain(m *testing.M) {
	roscmd := preprocess()
	code := m.Run()
	roscmd.postprocess()

	os.Exit(code)
}
