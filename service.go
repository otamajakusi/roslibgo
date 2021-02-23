package roslibgo

import (
	"encoding/json"
	"fmt"
)

type Service struct {
	ros          *Ros
	name         string
	serviceType  string
	isAdvertized bool
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#346-call-service
type ServiceCall struct {
	Op           string          `json:"op"`
	Id           string          `json:"id,omitempty"`
	Service      string          `json:"service"`
	Args         json.RawMessage `json:"args,omitempty"`
	FragmentSize int             `json:"fragment_size,omitempty"`
	Compression  string          `json:"compression,omitempty"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#347-advertise-service
type ServiceAdvertise struct {
	Op      string `json:"op"`
	Type    string `json:"type"`
	Service string `json:"service"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#348-unadvertise-service
type ServiceUnadvertise struct {
	Op      string `json:"op"`
	Service string `json:"service"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#349-service-response
type ServiceResponse struct {
	Op      string          `json:"op"`
	Id      string          `json:"id,omitempty"`
	Service string          `json:"service"`
	Values  json.RawMessage `json:"values,omitempty"`
	Result  bool            `json:"result"`
}

type ServiceCallback func(json.RawMessage) (bool, json.RawMessage)

func NewService(ros *Ros, name string, serviceType string) *Service {
	service := Service{ros: ros, name: name, serviceType: serviceType, isAdvertized: false}
	return &service
}

func (service *Service) Call() {
}

func (service *Service) Advertise(callback ServiceCallback) {
	ros := service.ros
	ros.initializeMessage(service.name)

	srvAdv := ServiceAdvertise{Op: "advertise_service", Type: service.serviceType, Service: service.name}
	err := ros.ws.writeJSON(srvAdv)
	if err != nil {
		fmt.Printf("Advertise: error %v\n", err)
	}
	go func() {
		for {
			srvCall := ros.loadMessage(service.name)
			result, values := callback(srvCall.(*ServiceCall).Args)
			id := srvCall.(*ServiceCall).Id
			srvResp := ServiceResponse{Op: "service_response", Id: id, Service: service.name, Values: values, Result: result}
			ros.ws.writeJSON(srvResp)
		}
	}()
}

func (service *Service) Unadvertise() {
	ros := service.ros
	srv := ServiceAdvertise{Op: "unadvertise_service", Service: service.name}
	err := ros.ws.writeJSON(srv)
	if err != nil {
		fmt.Printf("Unadvertise: error %v\n", err)
	}
}
