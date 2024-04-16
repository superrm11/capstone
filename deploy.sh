#!/bin/sh

rsync -av --delete "$(readlink -f $(pwd))" ryan@10.42.0.13:/home/ryan/
