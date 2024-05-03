#!/bin/sh

rsync -av --delete "$(readlink -f $(pwd))" jetson@10.42.0.137:/home/jetson/
# ssh ryan@10.42.0.13 "python3 /home/ryan/capstone/main.py"
