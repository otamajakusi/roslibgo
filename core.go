package roslibgo

import (
	"encoding/json"
	"fmt"
	"github.com/gorilla/websocket"
	"sync"
	//"time"
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

type Topic struct {
	ros          *Ros
	name         string
	messageType  string
	isAdvertized bool
}

type Base struct {
	Op string `json:"op"`
	Id string `json:"id"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#343-publish--publish-
const PublishOp = "publish"

type PublishMessage struct {
	Op    string          `json:"op"`
	Id    string          `json:"id,omitempty"`
	Topic string          `json:"topic"`
	Msg   json.RawMessage `json:"msg,omitempty"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#344-subscribe
const SubscribeOp = "subscribe"

type SubscribeMessage struct {
	Op           string `json:"op"`
	Id           string `json:"id,omitempty"`
	Topic        string `json:"topic"`
	Type         string `json:"type,omitempty"`
	ThrottleRate int    `json:"throttle_rate,omitempty"` //In msec
	QueueLength  int    `json:"queue_length,omitempty"`  //Default: 1
	FragmentSize int    `json:"fragment_size,omitempty"`
	Compression  string `json:"compression,omitempty"`
}

type TopicCallback func(json.RawMessage)

func (rosWs *RosWs) readMessage() ([]byte, error) {
	_, msg, err := rosWs.ws.ReadMessage()
	return msg, err
}

func (rosWs *RosWs) writeJSON(msg interface{}) error {
	rosWs.mutex.Lock()
	err := rosWs.ws.WriteJSON(msg)
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
	recv := func() {
		msg, err := ros.ws.readMessage()
		if err != nil {
			fmt.Printf("RunForever: error %v\n", err)
			return
		}
		var base Base
		json.Unmarshal(msg, &base)
		// fmt.Println(string(msg))
		switch base.Op {
		case "publish":
			var message PublishMessage
			json.Unmarshal(msg, &message)
			ros.storeMessage(PublishOp, message.Topic, &message)
		case "subscribe":
		case "unsubscribe":
		case "call_service":
			var service ServiceCall
			json.Unmarshal(msg, &service)
			ros.storeMessage(ServiceCallOp, service.Service, &service)
		case "service_response":
			var service ServiceResponse
			json.Unmarshal(msg, &service)
			ros.storeMessage(ServiceResponseOp, service.Service, &service)
		default:
		}
	}
	for {
		recv()
	}
}

func (ros *Ros) connect() (*websocket.Conn, error) {
	ws := ros.ws.ws
	if ws != nil {
		return ws, nil
	}
	dialer := websocket.Dialer{}
	ws, _, err := dialer.Dial(ros.url, nil)
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

func NewTopic(ros *Ros, name string, messageType string) *Topic {
	topic := Topic{ros: ros, name: name, messageType: messageType, isAdvertized: false}
	return &topic
}

func (topic *Topic) Publish(data json.RawMessage) {
	ros := topic.ros
	id := fmt.Sprintf("%s:%s:%d", PublishOp, topic.name, ros.incCounter())
	msg := PublishMessage{Op: "publish", Id: id, Topic: topic.name, Msg: data}
	err := ros.ws.writeJSON(msg)
	if err != nil {
		fmt.Printf("Publish: error %v\n", err)
	}
}

func (topic *Topic) Subscribe(callback TopicCallback) {
	ros := topic.ros
	ros.createMessage(SubscribeOp, topic.name)

	id := fmt.Sprintf("%s:%s:%d", SubscribeOp, topic.name, ros.incCounter())
	msg := SubscribeMessage{Op: "subscribe", Id: id, Topic: topic.name, Type: topic.messageType}
	err := ros.ws.writeJSON(msg)
	if err != nil {
		fmt.Printf("Subscribe: error %v\n", err)
	}
	go func() {
		for {
			callback((ros.retrieveMessage(SubscribeOp, topic.name)).(*PublishMessage).Msg)
		}
	}()
}

func (topic *Topic) Unsubscribe() {
}
