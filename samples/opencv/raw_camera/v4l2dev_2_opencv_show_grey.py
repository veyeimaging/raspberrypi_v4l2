import sys
import argparse
import subprocess
import cv2

def read_cam(width, height, fps):
    
    v4l2_cmd = f"v4l2-ctl --set-selection=target=crop,top=0,left=0,width={width},height={height}"
    subprocess.run(v4l2_cmd, shell=True)
    
    v4l2_cmd = f"v4l2-ctl --set-ctrl frame_rate={fps}"
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
    args = parser.parse_args()

    read_cam(args.width, args.height, args.fps)