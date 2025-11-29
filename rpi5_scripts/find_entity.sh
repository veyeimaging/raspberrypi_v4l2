#!/bin/bash

#set -x
g_cm5=0

I2CBUS_CAM1=11
I2CBUS_CAM0=10


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
check_rpi_board()
{
	model=$(tr -d '\0' </proc/device-tree/model)

	if [[ $model == *"Raspberry Pi 5"* ]]; then
		echo "This is a Raspberry Pi 5."
        g_cm5=0
    elif [[ $model == *"Raspberry Pi Compute Module 5"* ]]; then
        echo "This is a Raspberry Pi Compute Module 5."
        g_cm5=1
	else
		echo "This is not a Raspberry Pi 5. Will exit."
		exit 0;
	fi
}

check_i2c_bus() {

    kernel_version=$(uname -r | awk -F '+' '{print $1}') 

    ref_version="6.6.62"

    IFS='.' read -r k_major k_minor k_patch <<<"$kernel_version"
    IFS='.' read -r r_major r_minor r_patch <<<"$ref_version"
   # echo "ref version: $k_major $k_minor $k_patch"
   # echo "read version: $r_major $r_minor $r_patch"
    if ((k_major > r_major)) || \
       ((k_major == r_major && k_minor > r_minor)) || \
       ((k_major == r_major && k_minor == r_minor && k_patch >= r_patch)); then
        if ((g_cm5 == 1));then
            I2CBUS_CAM1=0
        else
            I2CBUS_CAM1=11
        fi
        I2CBUS_CAM0=10
        
    else
        if ((g_cm5 == 1));then
            I2CBUS_CAM1=0
        else
            I2CBUS_CAM1=4
        fi
        I2CBUS_CAM1=6
    fi
    echo "Kernel version is $kernel_version, use i2c-$I2CBUS_CAM0 for CAM0 and i2c-$I2CBUS_CAM1 for CAM1."
}

check_rpi_board;
check_i2c_bus;

find_entity_device "gxcam" $I2CBUS_CAM0
find_entity_device "gxcam" $I2CBUS_CAM1

find_entity_device "veyecam2m" $I2CBUS_CAM0
find_entity_device "veyecam2m" $I2CBUS_CAM1

find_entity_device "mvcam" $I2CBUS_CAM0
find_entity_device "mvcam" $I2CBUS_CAM1

find_entity_device "csimx307" $I2CBUS_CAM0
find_entity_device "csimx307" $I2CBUS_CAM1

find_entity_device "cssc132" $I2CBUS_CAM0
find_entity_device "cssc132" $I2CBUS_CAM1