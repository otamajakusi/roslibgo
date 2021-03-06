package roslibgo

import (
	"encoding/json"
	"fmt"
	"github.com/otamajakusi/recws"
	// "golang.org/x/net/websocket"
	"sync"
	"time"
)

var ErrNotConnected = recws.ErrNotConnected

type RosMessage struct {
	message map[string]chan interface{}
	mutex   sync.Mutex
}

type RosWs struct {
	ws    *recws.RecConn
	mutex sync.Mutex
}

type Ros struct {
	url         string
	ws          RosWs
	counter     int
	mutex       sync.Mutex
	message     RosMessage
	onConnected map[string]func()
}

type Base struct {
	Op string `json:"op"`
	Id string `json:"id"`
}

func (rosWs *RosWs) readMessage() ([]byte, error) {
	_, msg, err := rosWs.ws.ReadMessage()
	return msg, err
}

func (rosWs *RosWs) writeJSON(msg interface{}) error {
	err := rosWs.ws.WriteJSON(msg)
	if err != nil {
		fmt.Printf("writeJson %v\n", err)
	}
	return err
}

func NewRos(url string) (*Ros, error) {
	ros := Ros{url: url, ws: RosWs{ws: nil}}
	ros.message.message = make(map[string]chan interface{})
	ros.onConnected = make(map[string]func())
	ws := ros.connect()
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
			time.Sleep(time.Second * 1) // TODO: should be configurable
		}
	}
}

func (ros *Ros) registerOnConnected(key string, onConnect func()) {
	ros.mutex.Lock()
	ros.onConnected[key] = onConnect
	ros.mutex.Unlock()
}

func (ros *Ros) connect() *recws.RecConn {
	ws := ros.ws.ws
	if ws != nil {
		return ws
	}
	ws = &recws.RecConn{RecIntvlMin: 1, RecIntvlMax: 2, NonVerbose: true, SubscribeHandler: ros.onConnect} // TODO: should be configurable
	ws.Dial(ros.url, nil)
	return ws
}

func (ros *Ros) incCounter() int {
	ros.mutex.Lock()
	counter := ros.counter
	ros.counter++
	ros.mutex.Unlock()
	return counter
}

func (ros *Ros) onConnect() error {
	ros.mutex.Lock()
	for _, v := range ros.onConnected {
		v()
	}
	ros.mutex.Unlock()
	return nil
}

func (ros *Ros) createMessage(op string, name string) {
	ch := make(chan interface{})
	ros.message.mutex.Lock()
	ros.message.message[op+":"+name] = ch
	ros.message.mutex.Unlock()
}

func (ros *Ros) destroyMessage(op string, name string) {
	ros.message.mutex.Lock()
	close(ros.message.message[op+":"+name])
	ros.message.mutex.Unlock()
}

func (ros *Ros) storeMessage(op string, name string, value interface{}) {
	ros.message.mutex.Lock()
	ros.message.message[op+":"+name] <- value
	ros.message.mutex.Unlock()
}

func (ros *Ros) retrieveMessage(op string, name string) (interface{}, bool) {
	v, ok := <-ros.message.message[op+":"+name]
	return v, ok
}
