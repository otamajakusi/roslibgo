[![test](https://github.com/otamajakusi/roslibgo/actions/workflows/test.yaml/badge.svg)](https://github.com/otamajakusi/roslibgo/actions/workflows/test.yaml)

# roslibgo: ROS bridge client library with Go
roslibgo is the client library to connect [rosbridge](http://wiki.ros.org/rosbridge_suite) with Go-language.

# Feature
- Topic publishing
- Topic subscribing
- Service call (client)
- Service advertisement (server)
- reconnecting to ROS Bridge when connection is closed.
- support concurrent programming

# Installation
To install roslibgo
`go get github.com/otamajakusi/roslibgo`

# Example
See [example/main.go](example/main.go)
