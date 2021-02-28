package roslibgo

import (
	"encoding/json"
	"fmt"
	"github.com/gorilla/websocket"
	// "golang.org/x/net/websocket"
	"sync"
)

type RosMessage struct {
	message map[string]chan interface{}
	mutex   sync.Mutex
}

type RosWs struct {
	ws    *websocket.Conn
	mutex sync.Mutex
}

type Ros struct {
	url     string
	ws      RosWs
	counter int
	mutex   sync.Mutex
	message RosMessage
}

type Base struct {
	Op string `json:"op"`
	Id string `json:"id"`
}

func (rosWs *RosWs) readMessage() ([]byte, error) {
	// [gorilla websocket]
	_, msg, err := rosWs.ws.ReadMessage()
	// [x/net/websocket]
	// var msg []byte
	// err := websocket.Message.Receive(rosWs.ws, &msg)
	return msg, err
}

func (rosWs *RosWs) writeJSON(msg interface{}) error {
	rosWs.mutex.Lock()
	// [gorilla websocket]
	err := rosWs.ws.WriteJSON(msg)
	// [x/net/websocket]
	// err := websocket.JSON.Send(rosWs.ws, msg)
	rosWs.mutex.Unlock()
	return err
}

func NewRos(url string) (*Ros, error) {
	ros := Ros{url: url, ws: RosWs{ws: nil}}
	ros.message.message = make(map[string]chan interface{})
	ws, err := ros.connect()
	if err != nil {
		return nil, fmt.Errorf("NewRos: %v", err)
	}
	ros.ws.ws = ws
	return &ros, nil
}

func (ros *Ros) Run() {
	go ros.RunForever()
}

func (ros *Ros) RunForever() {
	recv := func() error {
		msg, err := ros.ws.readMessage()
		if err != nil {
			return err
		}
		var base Base
		json.Unmarshal(msg, &base)
		//fmt.Println(string(msg))
		switch base.Op {
		case "publish":
			var message PublishMessage
			json.Unmarshal(msg, &message)
			ros.storeMessage(PublishOp, message.Topic, &message)
		case "call_service":
			var service ServiceCall
			json.Unmarshal(msg, &service)
			ros.storeMessage(ServiceCallOp, service.Service, &service)
		case "service_response":
			var service ServiceResponse
			json.Unmarshal(msg, &service)
			ros.storeMessage(ServiceResponseOp, service.Service, &service)
		case "subscribe":
		case "unsubscribe":
		default:
		}
		return nil
	}
	for {
		err := recv()
		if err != nil {
			fmt.Printf("RunForever: error %v\n", err)
			break
		}
	}
}

func (ros *Ros) connect() (*websocket.Conn, error) {
	ws := ros.ws.ws
	if ws != nil {
		return ws, nil
	}
	// [gorilla websocket]
	dialer := websocket.Dialer{}
	ws, _, err := dialer.Dial(ros.url, nil)
	// [x/net/websocket]
	// ws, err := websocket.Dial(ros.url, "", "https://localhost")
	return ws, err
}

func (ros *Ros) incCounter() int {
	ros.mutex.Lock()
	counter := ros.counter
	ros.counter++
	ros.mutex.Unlock()
	return counter
}

func (ros *Ros) createMessage(op string, name string) {
	ch := make(chan interface{})
	ros.message.mutex.Lock()
	ros.message.message[op+":"+name] = ch
	ros.message.mutex.Unlock()
}

func (ros *Ros) destroyMessage(op string, name string) {
	ros.message.mutex.Lock()
	ros.message.message[op+":"+name] = nil
	ros.message.mutex.Unlock()
}

func (ros *Ros) storeMessage(op string, name string, value interface{}) {
	ros.message.mutex.Lock()
	ros.message.message[op+":"+name] <- value
	ros.message.mutex.Unlock()
}

func (ros *Ros) retrieveMessage(op string, name string) interface{} {
	return <-ros.message.message[op+":"+name]
}
