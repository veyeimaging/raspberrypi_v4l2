#!/bin/bash
gst-launch-1.0 v4l2src device=/dev/video0 ! "video/x-raw,format=(string)UYVY, width=(int)1920, height=(int)1080,framerate=(fraction)60/1" ! videoconvert ! autovideosink sync=false -v
