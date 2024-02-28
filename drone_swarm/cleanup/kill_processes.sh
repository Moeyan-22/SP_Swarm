#!/bin/bash

# Get the port number as a command-line argument
port=$1

# Check if a valid port number is provided
if [[ -z "$port" ]]; then
    echo "Usage: $0 <port_number>"
    exit 1
fi

# Find the PID using the provided port number
pid=$(lsof -ti :$port)

# Check if PID is found
if [[ -n "$pid" ]]; then
    echo "Terminating process with PID $pid using port $port"
    kill -9 $pid
else
    echo "No process found using port $port"
fi

