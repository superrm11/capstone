#!/bin/sh

 rsync -av --delete "$(readlink -f $(pwd))" jetson@10.3.39.2:/home/jetson/

