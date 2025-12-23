#!/bin/bash
gst-launch-1.0 v4l2src num-buffers=1 device=/dev/video0 ! 'video/x-raw, format=(string)UYVY, width=1920,height=1080' ! jpegenc ! filesink location=test_image.jpg
