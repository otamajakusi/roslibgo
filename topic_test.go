package roslibgo

import (
	"encoding/json"
	"github.com/stretchr/testify/assert"
	"os/exec"
	//"sync"
	//"fmt"
	"testing"
	"time"
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

func rosTopicPubCmd(topic_name string, topic_type string, topic_msg string) {
	cmd := exec.Command("rostopic", "pub", "-1", topic_name, topic_type, topic_msg)
	cmd.Run()
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
	s, _ := json.Marshal(StdMsgs_String{Data: "Hello roslibgo"})
	<-started // wait go routine started

	pub.Publish(json.RawMessage(s))
	assert.Equal(t, <-output, "data: \"Hello roslibgo\"\n---\n")
}

func TestPublishWithSubscribe(t *testing.T) {
	ros, _ := NewRos("ws://localhost:9090")
	ros.Run()

	// subscribe
	sub := NewTopic(ros, "/foo/bar", "std_msgs/String")
	output := make(chan string)
	callback := func(data json.RawMessage) {
		var msg StdMsgs_String
		json.Unmarshal(data, &msg)
		output <- msg.Data
	}
	sub.Subscribe(callback)
	time.Sleep(1 * time.Second)

	// publish
	pub := NewTopic(ros, "/foo/bar", "std_msgs/String")
	s, _ := json.Marshal(StdMsgs_String{Data: "Hello roslibgo"})
	pub.Publish(json.RawMessage(s))

	assert.Equal(t, <-output, "Hello roslibgo")
}

func TestSubscribeWithRosTopicPub(t *testing.T) {
	ros, _ := NewRos("ws://localhost:9090")
	ros.Run()

	// subscribe
	sub := NewTopic(ros, "/foo/bar", "std_msgs/String")
	output := make(chan string)
	callback := func(data json.RawMessage) {
		var msg StdMsgs_String
		json.Unmarshal(data, &msg)
		output <- msg.Data
	}
	sub.Subscribe(callback)
	time.Sleep(1 * time.Second)

	rosTopicPubCmd("/foo/bar", "std_msgs/String", "data: 'Hello roslibgo'")

	assert.Equal(t, <-output, "Hello roslibgo")
}
