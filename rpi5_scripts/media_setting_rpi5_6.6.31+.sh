#!/bin/bash

#set -x
#
I2CBUS_CAM1=4
I2CBUS_CAM0=6

#default params of YUV_type cameras
WIDTH_YUV=1920
HEIGHT_YUV=1080
MEDIA_FMT_YUV=UYVY8_1X16
PIXEL_FMT_YUV=UYVY

#default params of MV_type cameras
WIDTH_MV=1280
HEIGHT_MV=1024
MEDIA_FMT_MV=Y8_1X8
PIXEL_FMT_MV=GREY

g_camera_type=null
g_camera_name=null
g_camera_probed=0
g_media_device=null
g_video_device=null
g_video_subdevice=null

g_width=0
g_height=0
g_media_fmt=null
g_pixel_fmt=null
cam=2 #all cams

print_usage() {
    echo "Usage: $0 veyecam2m/csimx307/cssc132/mvcam -fmt [UYVY/RAW8/RAW10/RAW12] -w [width] -h [height] -c [cam 0|1]"
    echo -e "This shell script is designed to detect the connection of a camera on Raspberry Pi 5. \n
    It utilizes media-ctl and v4l2-ctl commands to configure the linking relationships and data formats of the media pad. \n
    Once completed, you can directly use /dev/video0 or /dev/video8 to obtain image data.\n"
}

check_camera_name() {
    case $1 in
        veyecam2m)
            g_camera_name=veyecam2m
			g_camera_type="YUV_type"
            ;;
        csimx307)
            g_camera_name=csimx307
			g_camera_type="YUV_type"
            ;;
        cssc132)
            g_camera_name=cssc132
			g_camera_type="YUV_type"
            ;;
        mvcam)
            g_camera_name=mvcam
			g_camera_type="MV_type"
            ;;
        *)
            echo "Please provide a correct camera module name!"
            exit 1
            ;;
    esac
	if [ "$g_camera_type" = "YUV_type" ]; then
		echo "camera is YUV type"
		g_width=$WIDTH_YUV
		g_height=$HEIGHT_YUV
		g_media_fmt=$MEDIA_FMT_YUV
		g_pixel_fmt=$PIXEL_FMT_YUV
	else
		g_width=$WIDTH_MV
		g_height=$HEIGHT_MV
		g_media_fmt=$MEDIA_FMT_MV
		g_pixel_fmt=$PIXEL_FMT_MV
	fi
}

parse_fmt()
{
	case $1 in
        UYVY)
			g_media_fmt=UYVY8_1X16
			g_pixel_fmt=UYVY
            ;;
        RAW8)
            g_media_fmt=Y8_1X8
			g_pixel_fmt=GREY
            ;;
        RAW10)
            g_media_fmt=Y10_1X10
			g_pixel_fmt='Y10P'
            ;;
        RAW12)
            g_media_fmt=Y12_1X12
			g_pixel_fmt='Y12P'
            ;;
        *)
            echo "format not supported!"
            exit 1
            ;;
    esac
}

parse_arguments() {

  while [ "$#" -gt 0 ]; do
    case "$1" in
      veyecam2m | csimx307 | cssc132 | mvcam)
        check_camera_name "$1"
        ;;
      -fmt)
        shift
		parse_fmt "$1"
        ;;
      -w)
        shift
        g_width="$1"
        ;;
      -h)
        shift
        g_height="$1"
        ;;
      -c)
	shift
	cam=$1
	;;
      *)
        echo "Unknown option: $1"
		print_usage
        exit 1
        ;;
    esac
    shift
  done
	
	echo "camera name $g_camera_name; width $g_width; height $g_height; media_fmt $g_media_fmt; pixel_fmt $g_pixel_fmt"
}

check_rpi_board()
{
	model=$(tr -d '\0' </proc/device-tree/model)

	if [[ $model == *"Raspberry Pi 5"* ]]; then
		echo "This is a Raspberry Pi 5."
	else
		echo "This is not a Raspberry Pi 5. Will exit."
		exit 0;
	fi
}

probe_camera_entity() 
{
    local entity_name="$1"
    local entity_id="$2"
	local media_dev
    for media_dev in /dev/media*; do
        #echo "Checking $g_media_device..."
        entity_info=$(media-ctl -d $media_dev -p 2>/dev/null | grep "$entity_name $entity_id-003b")
        if [ -n "$entity_info" ]; then
			g_media_device=$media_dev
			#echo "Found $entity_name $entity_id entity in $g_media_device"
			return 1
        fi
    done
	return 0
}

set_camera_entity() 
{
	#echo "$g_media_device $g_video_device"
	media-ctl -d $g_media_device -r
	#enable "rp1-cfe-csi2_ch0":0 [ENABLED]-->/dev/video0
	media-ctl -d $g_media_device -l ''\''csi2'\'':4 -> '\''rp1-cfe-csi2_ch0'\'':0 [1]'
	#set media's setting
	media-ctl -d "$g_media_device" --set-v4l2 "'$1 $2-003b':0[fmt:${g_media_fmt}/${g_width}x${g_height} field:none]"
	#media-ctl -d $g_media_device -V ''\''csi2'\'':0 [fmt:UYVY8_1X16/1920x1080 field:none]'
	media-ctl -d "$g_media_device" -V "'csi2':0 [fmt:${g_media_fmt}/${g_width}x${g_height} field:none]"
	#media-ctl -d $g_media_device -V ''\''csi2'\'':4 [fmt:UYVY8_1X16/1920x1080 field:none]'
	media-ctl -d "$g_media_device" -V "'csi2':4 [fmt:${g_media_fmt}/${g_width}x${g_height} field:none]"
	#set video node
	v4l2-ctl -d $g_video_device --set-fmt-video=width=$g_width,height=$g_height,pixelformat=$g_pixel_fmt,colorspace=rec709,ycbcr=rec709,xfer=rec709,quantization=lim-range
}
### here really begain!

check_rpi_board;

if [ "$#" -lt 1 ]; then
    print_usage
    exit 1
fi

parse_arguments "$@"

if [ $cam -gt 0 ]; then
	probe_camera_entity $g_camera_name $I2CBUS_CAM1

	if [ $? -eq 1 ]; then
		echo "CAM1 probed: media device is $g_media_device"
		g_video_device=$(media-ctl -e rp1-cfe-csi2_ch0 -d $g_media_device)
		g_video_subdevice=$(media-ctl -e "$g_camera_name $I2CBUS_CAM1-003b" -d $g_media_device)	
		set_camera_entity $g_camera_name $I2CBUS_CAM1
		echo "set CAM1 finish, plese get frame from $g_video_device and use $g_video_subdevice for camera setting"
	else 
		echo "$g_camera_name $I2CBUS_CAM1 NOT FOUND"
	fi
fi

if [ $cam -ne 1 ]; then
	probe_camera_entity $g_camera_name $I2CBUS_CAM0

	if [ $? -eq 1 ]; then
		echo "CAM0 probed:  media device is  $g_media_device"
		g_video_device=$(media-ctl -e rp1-cfe-csi2_ch0 -d $g_media_device)
		g_video_subdevice=$(media-ctl -e "$g_camera_name $I2CBUS_CAM0-003b" -d $g_media_device)
		set_camera_entity $g_camera_name $I2CBUS_CAM0
		echo "set CAM0 finish, plese get frame from $g_video_device and use $g_video_subdevice for camera setting"
	else 
		echo "$g_camera_name $I2CBUS_CAM0 NOT FOUND"
	fi
fi

exit 0
