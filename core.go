package roslibgo

import (
	"fmt"
	"github.com/gorilla/websocket"
)

type Ros struct {
	url         string
	ws          *websocket.Conn
	isConnected bool
}

type Topic struct {
	name         string
	messageType  string
	isAdvertized bool
}

type Service struct {
	name         string
	serviceType  string
	isAdvertized bool
}

func NewRos(url string) (*Ros, error) {
	ros := Ros{url: url, isConnected: false}
	ws, err := ros.connect()
	if err != nil {
		return nil, fmt.Errorf("NewRos: %v", err)
	}
	ros.ws = ws
	return &ros, nil
}

func (ros *Ros) connect() (*websocket.Conn, error) {
	if ros.isConnected {
		return ros.ws, nil
	}
	dialer := websocket.Dialer{}
	ws, _, err := dialer.Dial(ros.url, nil)
	return ws, err
}

func NewTopic(ros *Ros, name string, messageType string) *Topic {
	topic := Topic{name: name, messageType: messageType, isAdvertized: false}
	return &topic
}

func (topic *Topic) Publish() {
}

func (topic *Topic) Subscribe() {
}

func (topic *Topic) Unsubscribe() {
}

func NewService(ros *Ros, name string, serviceType string) *Service {
	service := Service{name: name, serviceType: serviceType, isAdvertized: false}
	return &service
}

func (service *Service) Call() {
}

func (service *Service) Advertize() {
}

func (service *Service) Unadvertize() {
}
