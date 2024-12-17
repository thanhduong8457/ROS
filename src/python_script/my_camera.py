import cv2
import time

# GStreamer Pipeline to access the Raspberry Pi camera
# GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

#cap = cv2.VideoCapture('Resources/test.mkv')
cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

while True:
    success, img = cap.read()
    img = cv2.resize(img, (0,0), None, 0.5, 0.5) # reduce amount process of program

    cv2.imshow("Result", img)   #when run this program we can delete this command

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    