package main

import (
	"../../roslibgo"
	"encoding/json"
	"fmt"
	"time"
)

type StdMsgs_String struct {
	Data string `json:"data"`
}

func callbackA(data json.RawMessage) {
	var msg StdMsgs_String
	json.Unmarshal(data, &msg)
	fmt.Println("a:", msg.Data)
}

func callbackB(data json.RawMessage) {
	var msg StdMsgs_String
	json.Unmarshal(data, &msg)
	fmt.Println("b:", msg.Data)
}

func main() {
	ros, _ := roslibgo.NewRos("ws://localhost:9090")
	ros.Run()

	suba := roslibgo.NewTopic(ros, "/a", "std_msgs/String")
	suba.Subscribe(callbackA)
	subb := roslibgo.NewTopic(ros, "/b", "std_msgs/String")
	subb.Subscribe(callbackB)

	pubLoop := func(name string, str string) func() {
		i := 0
		pub := roslibgo.NewTopic(ros, name, "std_msgs/String")
		return func() {
			for {
				//fmt.Printf("--- %s: %d\n", str, i)
				s, _ := json.Marshal(StdMsgs_String{Data: fmt.Sprintf("%s: %d", str, i)})
				i++
				pub.Publish(json.RawMessage(s))
				time.Sleep(10 * time.Millisecond)
			}
		}
	}
	a := pubLoop("/a", "hoge")
	b := pubLoop("/b", "fuga")
	go a()
	go b()
	select {}
}
