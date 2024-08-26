#!/bin/bash

protoc -I=src/messages --cpp_out=src/main/messages_include src/messages/*.proto