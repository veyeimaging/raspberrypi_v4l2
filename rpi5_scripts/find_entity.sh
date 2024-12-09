#!/bin/bash

#set -x
media_device="/dev/media0"
# 寻找包含 "veyecam2m 4-003b" entity 的设备节点
I2CBUS_CAM1=4
I2CBUS_CAM0=6

check_rpi_board()
{
	model=$(tr -d '\0' </proc/device-tree/model)

    if [[ $model == *"Raspberry Pi 5"* ]]; then
        echo "This is a Raspberry Pi 5."
        echo "Please use i2c-4 for cam1, i2c-6 for cam0"
        I2CBUS_CAM1=4
        I2CBUS_CAM0=6
    elif [[ $model == *"Raspberry Pi Compute Module 5"* ]]; then
        echo "This is a Raspberry Pi Compute Module 5."
        echo "Please use i2c-0 for cam1, i2c-6 for cam0"
        I2CBUS_CAM1=0
        I2CBUS_CAM0=6
    else
        echo "This is not a Raspberry Pi 5."
        exit 0
    fi
}

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

check_rpi_board

find_entity_device "veyecam2m" $I2CBUS_CAM0
find_entity_device "veyecam2m" $I2CBUS_CAM1

find_entity_device "mvcam" $I2CBUS_CAM0
find_entity_device "mvcam" $I2CBUS_CAM1

find_entity_device "csimx307" $I2CBUS_CAM0
find_entity_device "csimx307" $I2CBUS_CAM1

find_entity_device "cssc132" $I2CBUS_CAM0
find_entity_device "cssc132" $I2CBUS_CAM1