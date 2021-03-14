FROM osrf/ros:melodic-desktop-full

RUN apt update && apt install wget
RUN wget https://dl.google.com/go/go1.13.9.linux-amd64.tar.gz
RUN tar zxf go1.13.9.linux-amd64.tar.gz -C /usr/local
ENV PATH $PATH:/usr/local/go/bin
RUN apt install -y ros-melodic-rosbridge-server

RUN mkdir /roslibgo
WORKDIR /roslibgo
COPY . /roslibgo
