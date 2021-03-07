package roslibgo

import (
	"encoding/json"
	"github.com/stretchr/testify/assert"
	"os/exec"
	//"sync"
	"testing"
)

type StdMsgs_String struct {
	Data string `json:"data"`
}

func rosTopicEchoCmd(topic_name string) (chan int, chan string) {
	cmd := exec.Command("rostopic", "echo", "-n", "1", topic_name)
	output := make(chan string)
	started := make(chan int)
	go func() {
		started <- 1
		out, err := cmd.Output()
		if err != nil {
			output <- err.Error()
		} else {
			output <- string(out)
		}
	}()
	return started, output
}

// 1. (external process) launch rostopic echo
// 2. publish message
// 3. (external process) receive rostopic echo output
// 4. compare 2 and 3
func TestPublishWithRosTopicEcho(t *testing.T) {

	ros, _ := NewRos("ws://localhost:9090")
	ros.Run()

	started, output := rosTopicEchoCmd("/foo/bar")

	pub := NewTopic(ros, "/foo/bar", "std_msgs/String")
	s, _ := json.Marshal(StdMsgs_String{Data: "Hello world!"})
	<-started // wait echo process started

	pub.Publish(json.RawMessage(s))
	assert.Equal(t, <-output, "data: \"Hello world!\"\n---\n")
}

func TestSubscribe(t *testing.T) {
}

func TestUnsubscribe(t *testing.T) {
}
