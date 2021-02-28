package roslibgo

import (
	"encoding/json"
	"fmt"
)

type Topic struct {
	ros          *Ros
	name         string
	messageType  string
	isAdvertised bool
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#341-advertise--advertise-
const AdvertiseOp = "advertise"

type AdvertiseMessage struct {
	Op    string `json:"op"`
	Id    string `json:"id,omitempty"`
	Topic string `json:"topic"`
	Type  string `json:"type"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#342-unadvertise--unadvertise-
const UnadvertiseOp = "unadvertise"

type UnadvertiseMessage struct {
	Op    string `json:"op"`
	Id    string `json:"id,omitempty"`
	Topic string `json:"topic"`
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
	topic := Topic{ros: ros, name: name, messageType: messageType, isAdvertised: false}
	return &topic
}

func (topic *Topic) Publish(data json.RawMessage) error {
	err := topic.Advertise()
	if err != nil {
		return err
	}
	ros := topic.ros
	id := fmt.Sprintf("%s:%s:%d", PublishOp, topic.name, ros.incCounter())
	msg := PublishMessage{Op: "publish", Id: id, Topic: topic.name, Msg: data}
	return ros.ws.writeJSON(msg)
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
	id := fmt.Sprintf("%s:%s:%d", UnsubscribeOp, topic.name, topic.ros.incCounter())
	msg := SubscribeMessage{Op: UnsubscribeOp, Id: id, Topic: topic.name}
	return topic.ros.ws.writeJSON(msg)
}

func (topic *Topic) subscribe() error {
	id := fmt.Sprintf("%s:%s:%d", SubscribeOp, topic.name, topic.ros.incCounter())
	msg := SubscribeMessage{Op: SubscribeOp, Id: id, Topic: topic.name, Type: topic.messageType}
	err := topic.ros.ws.writeJSON(msg)
	if err != nil {
		topic.isAdvertised = true
	}
	return err
}

func (topic *Topic) Advertise() error {
	if topic.isAdvertised {
		return nil // do nothing
	}
	id := fmt.Sprintf("%s:%s:%d", AdvertiseOp, topic.name, topic.ros.incCounter())
	msg := AdvertiseMessage{Op: AdvertiseOp, Id: id, Type: topic.messageType, Topic: topic.name}
	err := topic.ros.ws.writeJSON(msg)
	if err != nil {
		topic.isAdvertised = true
	}
	return err
}

func (topic *Topic) Unadvertise() error {
	if !topic.isAdvertised {
		return nil // do nothing
	}
	id := fmt.Sprintf("%s:%s:%d", UnadvertiseOp, topic.name, topic.ros.incCounter())
	msg := UnadvertiseMessage{Op: UnadvertiseOp, Id: id, Topic: topic.name}
	err := topic.ros.ws.writeJSON(msg)
	if err != nil {
		topic.isAdvertised = false
	}
	return err
}
