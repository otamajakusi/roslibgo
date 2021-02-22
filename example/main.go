package main

import (
	"../../roslibgo"
	"encoding/json"
	"fmt"
)

type StdMsgs_String struct {
	Data string `json:"data"`
}

func callback(data json.RawMessage) {
	var msg StdMsgs_String
	json.Unmarshal(data, &msg)
	fmt.Println(msg.Data)
}

func main() {
	ros, _ := roslibgo.NewRos("ws://localhost:9090")
	ros.Run()

	topic := roslibgo.NewTopic(ros, "/foo/bar", "std_msgs/String")
	topic.Subscribe(callback)

	s, _ := json.Marshal(StdMsgs_String{Data: "piyo"})
	topic.Publish(json.RawMessage(s))
	select {}
}
