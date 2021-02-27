package roslibgo

import (
	"encoding/json"
	"fmt"
)

type Topic struct {
	ros          *Ros
	name         string
	messageType  string
	isAdvertized bool
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

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#345-unsubscribe
const UnsubscribeOp = "unsubscribe"

type UnsubscribeMessage struct {
	Op    string `json:"op"`
	Id    string `json:"id,omitempty"`
	Topic string `json:"topic"`
}

type TopicCallback func(json.RawMessage)

func NewTopic(ros *Ros, name string, messageType string) *Topic {
	topic := Topic{ros: ros, name: name, messageType: messageType, isAdvertized: false}
	return &topic
}

func (topic *Topic) Publish(data json.RawMessage) error {
	ros := topic.ros
	id := fmt.Sprintf("%s:%s:%d", PublishOp, topic.name, ros.incCounter())
	msg := PublishMessage{Op: "publish", Id: id, Topic: topic.name, Msg: data}
	err := ros.ws.writeJSON(msg)
	return err
}

func (topic *Topic) Subscribe(callback TopicCallback) error {
	err := topic.subscribe()
	if err != nil {
		return err
	}
	go func() {
		ros := topic.ros
		ros.createMessage(PublishOp, topic.name)
		defer ros.destroyMessage(PublishOp, topic.name)
		for {
			callback((ros.retrieveMessage(PublishOp, topic.name)).(*PublishMessage).Msg)
		}
	}()
	return nil
}

func (topic *Topic) Unsubscribe() error {
	ros := topic.ros
	id := fmt.Sprintf("%s:%s:%d", UnsubscribeOp, topic.name, ros.incCounter())
	msg := SubscribeMessage{Op: UnsubscribeOp, Id: id, Topic: topic.name}
	err := ros.ws.writeJSON(msg)
	return err
}

func (topic *Topic) subscribe() error {
	ros := topic.ros
	id := fmt.Sprintf("%s:%s:%d", SubscribeOp, topic.name, ros.incCounter())
	msg := SubscribeMessage{Op: SubscribeOp, Id: id, Topic: topic.name, Type: topic.messageType}
	return ros.ws.writeJSON(msg)
}
