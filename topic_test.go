package roslibgo

import (
	"fmt"
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
	roslaunch := exec.Command("roslaunch", "rosbridge_server", "rosbridge_websocket.launch")
	roslaunch.SysProcAttr = &syscall.SysProcAttr{Setpgid: true}
	roscore.Start()
	time.Sleep(100 * time.Millisecond)
	roslaunch.Start()
	return &rosCmd{roscore: roscore, roslaunch: roslaunch}
}

func (roscmd rosCmd) postprocess() {
	syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGINT)
	// syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGKILL)
	roscmd.roslaunch.Wait()
	syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGINT)
	// syscall.Kill(-roscmd.roscore.Process.Pid, syscall.SIGKILL)
	roscmd.roscore.Wait()
	fmt.Println("postprocess done")
}

func TestPublish(t *testing.T) {
	roscmd := preprocess()
	defer roscmd.postprocess()
	time.Sleep(time.Second)
}

func TestSubscribe(t *testing.T) {
}

func TestUnsubscribe(t *testing.T) {
}
