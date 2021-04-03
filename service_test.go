package roslibgo

import (
	"encoding/json"
	"github.com/stretchr/testify/assert"
	"os/exec"
	//"sync"
	//"fmt"
	"testing"
	//"time"
)

// std_srvs/SetBool
type StdSrvs_SetBoolRequest struct {
	Data bool `json:"data"`
}

type StdSrvs_SetBoolResponse struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
}

func TestServiceAndCall(t *testing.T) {
	ros, _ := NewRos("ws://localhost:9090")
	ros.Run()

	serv := NewService(ros, "/foo/bar", "std_srvs/SetBool")
	call := NewService(ros, "/foo/bar", "std_srvs/SetBool")

	servCb := func(request json.RawMessage) (bool, json.RawMessage) {
		var req StdSrvs_SetBoolRequest
		json.Unmarshal(request, &req)
		assert.Equal(t, req.Data, true)
		res := StdSrvs_SetBoolResponse{Success: true, Message: "Hello roslibgo"}
		raw, _ := json.Marshal(res)
		return true, raw
	}

	serv.Advertise(servCb)
	data, _ := json.Marshal(StdSrvs_SetBoolRequest{Data: true})
	response, err := call.Call(json.RawMessage(data))
	assert.Equal(t, err, nil)
	var res StdSrvs_SetBoolResponse
	json.Unmarshal(response, &res)
	assert.Equal(t, res, StdSrvs_SetBoolResponse{Success: true, Message: "Hello roslibgo"})
}

func rosServiceCallCmd(service_name string, value string) (string, error) {
	cmd := exec.Command("rosservice", "call", service_name, value)
	out, err := cmd.Output()
	return string(out), err
}

// call service from `rossrv call`
func TestServiceWithRosServiceCall(t *testing.T) {
	ros, _ := NewRos("ws://localhost:9090")
	ros.Run()

	serv := NewService(ros, "/foo/bar", "std_srvs/SetBool")
	servCb := func(request json.RawMessage) (bool, json.RawMessage) {
		var req StdSrvs_SetBoolRequest
		json.Unmarshal(request, &req)
		assert.Equal(t, req.Data, true)
		res := StdSrvs_SetBoolResponse{Success: true, Message: "Hello roslibgo"}
		raw, _ := json.Marshal(res)
		return true, raw
	}

	serv.Advertise(servCb)

	out, err := rosServiceCallCmd("/foo/bar", "true")
	assert.Equal(t, err, nil)
	assert.Equal(t, out, "success: True\nmessage: \"Hello roslibgo\"\n")
}
