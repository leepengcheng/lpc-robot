@echo off
SET B0_HOST_ID=192.168.6.90
start "ZMQ SERVER" .\b0_resolver.exe

start "Node"        .\vrepZMQ.exe
