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
const ServiceCallOp = "call_service"

type ServiceCall struct {
	Op           string          `json:"op"`
	Id           string          `json:"id,omitempty"`
	Service      string          `json:"service"`
	Args         json.RawMessage `json:"args,omitempty"`
	FragmentSize int             `json:"fragment_size,omitempty"` // not supported
	Compression  string          `json:"compression,omitempty"`   // not supported
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#347-advertise-service
const ServiceAdvertiseOp = "advertise_service"

type ServiceAdvertise struct {
	Op      string `json:"op"`
	Type    string `json:"type"`
	Service string `json:"service"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#348-unadvertise-service
const ServiceUnadvertiseOp = "unadvertise_service"

type ServiceUnadvertise struct {
	Op      string `json:"op"`
	Service string `json:"service"`
}

// https://github.com/biobotus/rosbridge_suite/blob/master/ROSBRIDGE_PROTOCOL.md#349-service-response
const ServiceResponseOp = "service_response"

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

func (service *Service) Call(request json.RawMessage) json.RawMessage {
	service.call(request)
	ros := service.ros
	ros.createMessage(ServiceResponseOp, service.name)
	defer ros.destroyMessage(ServiceResponseOp, service.name)
	return ros.retrieveMessage(ServiceResponseOp, service.name).(*ServiceResponse).Values
}

func (service *Service) Advertise(callback ServiceCallback) {
	service.advertise()

	ros := service.ros
	ros.createMessage(ServiceCallOp, service.name)

	go func() {
		for {
			srvCall := ros.retrieveMessage(ServiceCallOp, service.name)
			result, values := callback(srvCall.(*ServiceCall).Args)
			id := srvCall.(*ServiceCall).Id
			srvResp := ServiceResponse{Op: ServiceResponseOp, Id: id, Service: service.name, Values: values, Result: result}
			ros.ws.writeJSON(srvResp)
		}
	}()
}

func (service *Service) Unadvertise() {
	srv := ServiceAdvertise{Op: ServiceUnadvertiseOp, Service: service.name}
	err := service.ros.ws.writeJSON(srv)
	if err != nil {
		fmt.Printf("Unadvertise: error %v\n", err)
	}
}

func (service *Service) call(request json.RawMessage) {
	ros := service.ros
	id := fmt.Sprintf("ServiceCallOp:%s:%d", service.name, ros.incCounter())
	srv := ServiceCall{Op: ServiceCallOp, Id: id, Service: service.name, Args: request}
	err := service.ros.ws.writeJSON(srv)
	if err != nil {
		fmt.Printf("Call: error %v\n", err)
	}
}

func (service *Service) advertise() {
	srv := ServiceAdvertise{Op: ServiceAdvertiseOp, Type: service.serviceType, Service: service.name}
	err := service.ros.ws.writeJSON(srv)
	if err != nil {
		fmt.Printf("Advertise: error %v\n", err)
	}
}
