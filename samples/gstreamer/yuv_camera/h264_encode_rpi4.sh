#!/bin/sh


if [ "$1" != "0" ] && [ "$1" != "1" ]; then
    echo "Usage: $0 <0|1>"
    echo "  0 : encode to MP4"
    echo "  1 : encode to MKV"
    exit 1
fi

DEVICE="/dev/video0"
FORMAT="UYVY"
WIDTH=1920
HEIGHT=1080
FPS="30/1"
BITRATE=6200000
BUFFERS=300


PIPELINE="gst-launch-1.0 -e v4l2src io-mode=dmabuf device=$DEVICE num-buffers=$BUFFERS ! \"video/x-raw,format=(string)$FORMAT,width=(int)$WIDTH,height=(int)$HEIGHT,framerate=(fraction)$FPS\" ! v4l2h264enc capture-io-mode=dmabuf output-io-mode=dmabuf extra-controls=\"controls,h264_profile=4,video_bitrate=$BITRATE\" ! \"video/x-h264,profile=high,level=(string)4\" ! h264parse"

if [ "$1" = "0" ]; then
    OUTPUT="video.mp4"
    PIPELINE="$PIPELINE ! mp4mux ! filesink location=$OUTPUT"
elif [ "$1" = "1" ]; then
    OUTPUT="output.mkv"
    PIPELINE="$PIPELINE ! matroskamux ! filesink location=$OUTPUT"
fi

echo "Encoding to $OUTPUT..."
eval $PIPELINE

