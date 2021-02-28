package roslibgo

import (
	"encoding/json"
	"fmt"
	"github.com/stretchr/testify/assert"
	"os/exec"
	//"sync"
	"syscall"
	"testing"
	"time"
)

type rosCmd struct {
	roscore   *exec.Cmd
	roslaunch *exec.Cmd
}

type StdMsgs_String struct {
	Data string `json:"data"`
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
	return &rosCmd{roscore: nil, roslaunch: roslaunch}
}

func (roscmd rosCmd) postprocess() {
	// kill roscore
	syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGINT)
	// syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGKILL)
	roscmd.roslaunch.Wait()

	// kill roslaunch
	syscall.Kill(-roscmd.roslaunch.Process.Pid, syscall.SIGINT)
	// syscall.Kill(-roscmd.roscore.Process.Pid, syscall.SIGKILL)
	roscmd.roscore.Wait()
	fmt.Println("postprocess done")
}

func rosTopicEchoCmd(topic_name string) (chan int, chan string) {
	cmd := exec.Command("rostopic", "echo", "-n", "1", topic_name)
	output := make(chan string)
	started := make(chan int)
	go func() {
		fmt.Println("start echo")
		started <- 1
		out, err := cmd.Output()
		fmt.Println("output", string(out))
		if err != nil {
			output <- err.Error()
		} else {
			output <- string(out)
		}
	}()
	return started, output
}

func TestPublishWithRosTopicEcho(t *testing.T) {
	roscmd := preprocess()
	defer roscmd.postprocess()

	ros, _ := NewRos("ws://localhost:9090")
	ros.Run()

	fmt.Println("echocmd")
	started, echo := rosTopicEchoCmd("/foo/bar")
	fmt.Println("echocmd run")

	pub := NewTopic(ros, "/foo/bar", "std_msgs/String")
	s, _ := json.Marshal(StdMsgs_String{Data: "Hello world!"})
	fmt.Println("publish...")
	time.Sleep(10 * time.Millisecond)
	<-started // wait echo started
	for i := 0; i < 3; i++ {
		pub.Publish(json.RawMessage(s))
		time.Sleep(1 * time.Second)
	}
	fmt.Println("published")

	fmt.Println("wait chan")
	assert.Equal(t, <-echo, "data: \"Hello world!\"\n---\n")
}

func TestSubscribe(t *testing.T) {
}

func TestUnsubscribe(t *testing.T) {
}
