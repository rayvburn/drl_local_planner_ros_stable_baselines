#!/bin/bash

tmux kill-server
docker kill $(docker ps -q)
