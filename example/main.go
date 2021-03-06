package main

import (
	"encoding/json"
	"fmt"
	"github.com/otamajakusi/roslibgo"
	//"github.com/pkg/profile"
	"sync"
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
	//defer profile.Start(profile.MemProfile, profile.ProfilePath(".")).Stop()
	var wg sync.WaitGroup

	ros, _ := roslibgo.NewRos("ws://localhost:9090")
	ros.Run()

	// srv := roslibgo.NewService(ros, "/x", "std_srvs/SetBool")
	// call := roslibgo.NewService(ros, "/x", "std_srvs/SetBool")
	// srv.Advertise(serviceCallback)
	// data, _ := json.Marshal(StdSrvs_SetBoolRequest{Data: true})
	// callLoop := func() {
	// 	for i := 0; i < 10; i++ {
	// 		resp, err := call.Call(json.RawMessage(data))
	// 		if err != nil {
	// 			fmt.Println("error:", err)
	// 			break
	// 		}
	// 		fmt.Println("callback:", string(resp))
	// 		time.Sleep(10 * time.Millisecond)
	// 	}
	// }
	// go callLoop()

	suba := roslibgo.NewTopic(ros, "/a", "std_msgs/String")
	suba.Subscribe(callbackA)
	subb := roslibgo.NewTopic(ros, "/b", "std_msgs/String")
	subb.Subscribe(callbackB)

	pubLoop := func(name string, str string) func() {
		pub := roslibgo.NewTopic(ros, name, "std_msgs/String")
		return func() {
			for i := 0; ; i++ {
				//fmt.Printf("--- %s: %d\n", str, i)
				s, _ := json.Marshal(StdMsgs_String{Data: fmt.Sprintf("%s: %d", str, i)})
				pub.Publish(json.RawMessage(s))
				//_, _ = pub, s
				time.Sleep(1 * time.Millisecond)
			}
			wg.Done()
		}
	}
	a := pubLoop("/a", "hoge")
	b := pubLoop("/b", "fuga")
	wg.Add(1)
	go a()
	wg.Add(1)
	go b()
	wg.Wait()
}
