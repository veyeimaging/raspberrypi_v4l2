import sys
import argparse
import subprocess
import cv2

def read_cam(width, height, fps, ctldev):
    
    v4l2_cmd = f"v4l2-ctl -d {ctldev} --set-ctrl roi_x=0"
    subprocess.run(v4l2_cmd, shell=True)
    v4l2_cmd = f"v4l2-ctl -d {ctldev} --set-ctrl roi_y=0"
    subprocess.run(v4l2_cmd, shell=True)
    v4l2_cmd = f"v4l2-ctl -d /dev/video0 --set-fmt-video=width={width},height={height}"
    subprocess.run(v4l2_cmd, shell=True)
    v4l2_cmd = f"v4l2-ctl -d {ctldev} --set-ctrl frame_rate={fps}"
    subprocess.run(v4l2_cmd, shell=True)
    
    cap = cv2.VideoCapture(f"v4l2src io-mode=dmabuf device=/dev/video0 ! video/x-raw, format=(string)GRAY8, width=(int){width}, height=(int){height} ! appsink")
    if cap.isOpened():
        cv2.namedWindow("demo", cv2.WINDOW_AUTOSIZE)
        while True:
            ret_val, img = cap.read();
            cv2.imshow('demo',img)
            cv2.waitKey(1)
    else:
     print ("camera open failed");

    cv2.destroyAllWindows()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Read camera video stream')
    parser.add_argument('--width', type=int, default=1080, help='width of the video stream')
    parser.add_argument('--height', type=int, default=1080, help='height of the video stream')
    parser.add_argument('--fps', type=int, default=30, help='fps of the video stream')
    parser.add_argument('--ctldev', type=str, default='/dev/video0', help='For rpi5 only,subdevice for param setting')
    args = parser.parse_args()

    read_cam(args.width, args.height, args.fps, args.ctldev)