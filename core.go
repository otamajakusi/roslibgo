package roslibgo

import (
	"encoding/json"
	"fmt"
	"github.com/gorilla/websocket"
	"sync"
)

type Ros struct {
	url      string
	ws       *websocket.Conn
	counter  int
	mutex    sync.Mutex
	received map[string]chan interface{}
}

type Topic struct {
	ros          *Ros
	name         string
	messageType  string
	isAdvertized bool
}

type Service struct {
	ros          *Ros
	name         string
	serviceType  string
	isAdvertized bool
}

type Base struct {
	Op string `json:"op"`
	Id string `json:"id"`
}

// https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md#343-publish--publish-
type PublishMessage struct {
	Op    string          `json:"op"`
	Id    string          `json:"id,omitempty"`
	Topic string          `json:"topic"`
	Msg   json.RawMessage `json:"msg,omitempty"`
}

// https://github.com/RobotWebTools/rosbridge_suite/blob/groovy-devel/ROSBRIDGE_PROTOCOL.md#344-subscribe
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

func NewRos(url string) (*Ros, error) {
	ros := Ros{url: url, ws: nil}
	ros.received = make(map[string]chan interface{})
	ws, err := ros.connect()
	if err != nil {
		return nil, fmt.Errorf("NewRos: %v", err)
	}
	ros.ws = ws
	return &ros, nil
}

func (ros *Ros) Run() {
	go ros.RunForever()
}

func (ros *Ros) RunForever() {
	recv := func() {
		_, msg, err := ros.ws.ReadMessage()
		if err != nil {
			fmt.Printf("RunForever: error %v\n", err)
			return
		}
		var base Base
		json.Unmarshal(msg, &base)
		fmt.Println(string(msg))
		switch base.Op {
		case "publish":
			var message PublishMessage
			json.Unmarshal(msg, &message)
			ros.received[message.Topic] <- &message // write messge to the channel
		case "subscribe":
		case "unsubscribe":
		case "call_service":
		case "service_response":
		default:
		}
	}
	for {
		recv()
	}
}

func (ros *Ros) connect() (*websocket.Conn, error) {
	if ros.ws != nil {
		return ros.ws, nil
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

func NewTopic(ros *Ros, name string, messageType string) *Topic {
	topic := Topic{ros: ros, name: name, messageType: messageType, isAdvertized: false}
	return &topic
}

func (topic *Topic) Publish(data json.RawMessage) {
	id := fmt.Sprintf("publish:%s:%d", topic.name, topic.ros.incCounter())
	msg := PublishMessage{Op: "publish", Id: id, Topic: topic.name, Msg: data}
	err := topic.ros.ws.WriteJSON(msg)
	if err != nil {
		fmt.Printf("PUblish: error %v\n", err)
	}
}

func (topic *Topic) Subscribe(callback TopicCallback) {
	ch := make(chan interface{})
	topic.ros.received[topic.name] = ch

	id := fmt.Sprintf("subscribe:%s:%d", topic.name, topic.ros.incCounter())
	msg := SubscribeMessage{Op: "subscribe", Id: id, Topic: topic.name, Type: topic.messageType}
	err := topic.ros.ws.WriteJSON(msg)
	if err != nil {
		fmt.Printf("Subscribe: error %v\n", err)
	}
	go func() {
		for {
			callback((<-ch).(*PublishMessage).Msg)
		}
	}()
}

func (topic *Topic) Unsubscribe() {
}

func NewService(ros *Ros, name string, serviceType string) *Service {
	service := Service{ros: ros, name: name, serviceType: serviceType, isAdvertized: false}
	return &service
}

func (service *Service) Call() {
}

func (service *Service) Advertize() {
}

func (service *Service) Unadvertize() {
}
