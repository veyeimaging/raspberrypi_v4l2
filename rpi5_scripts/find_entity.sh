#!/bin/bash

#set -x
media_device="/dev/media0"
# 寻找包含 "veyecam2m 4-003b" entity 的设备节点

find_entity_device() {
    local entity_name="$1"
    local entity_id="$2"

    for media_device in /dev/media*; do
        #echo "Checking $media_device..."
        entity_info=$(media-ctl -d $media_device -p 2>/dev/null | grep "$entity_name $entity_id-003b")
        if [ -n "$entity_info" ]; then
            echo "Found $entity_name @ i2c-$entity_id entity on $media_device"
			g_video_device=$(media-ctl -e rp1-cfe-csi2_ch0 -d $media_device)
			g_video_subdevice=$(media-ctl -e "$entity_name $entity_id-003b" -d $media_device)
			echo "Plese get frame from $g_video_device and use $g_video_subdevice for camera setting."
        fi
    done
}

find_entity_device "veyecam2m" "4"
find_entity_device "veyecam2m" "6"

find_entity_device "mvcam" "4"
find_entity_device "mvcam" "6"

find_entity_device "csimx307" "4"
find_entity_device "csimx307" "6"

find_entity_device "cssc132" "4"
find_entity_device "cssc132" "6"