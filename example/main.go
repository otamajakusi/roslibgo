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

// std_srvs/SetBool
type StdSrvs_SetBoolRequest struct {
	Data bool `json:"data"`
}

type StdSrvs_SetBoolResponse struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
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

func serviceCallback(request json.RawMessage) (bool, json.RawMessage) {
	var req StdSrvs_SetBoolRequest
	json.Unmarshal(request, &req)
	res := StdSrvs_SetBoolResponse{Success: true, Message: "I'm OK"}
	raw, _ := json.Marshal(res)
	return true, raw
}

func main() {
	ros, _ := roslibgo.NewRos("ws://localhost:9090")
	ros.Run()

	srv := roslibgo.NewService(ros, "/x", "std_srvs/SetBool")
	call := roslibgo.NewService(ros, "/x", "std_srvs/SetBool")
	srv.Advertise(serviceCallback)
	data, _ := json.Marshal(StdSrvs_SetBoolRequest{Data: true})
	callLoop := func() {
		for {
			resp := call.Call(json.RawMessage(data))
			fmt.Println("callback:", string(resp))
			time.Sleep(10 * time.Millisecond)
		}
	}
	go callLoop()
	select {}

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
