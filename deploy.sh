#!/bin/sh

rsync -av --delete "$(readlink -f $(pwd))" ryan@10.42.0.13:/home/ryan/
# ssh ryan@10.42.0.13 "python3 /home/ryan/capstone/main.py"
