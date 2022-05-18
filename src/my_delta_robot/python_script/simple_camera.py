import cv2
import time
import numpy as np

m = 1.0
k1 = 0.0
k2 = 0.0

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver
    
def add_pos(a,b,c,d,arr,e):
    arr0 = []
    arr0.append(0.4585*(float(a+c//2)*m-k1)-212.2)
    arr0.append(237.7-0.3897*(float(b+d//2)))
    arr0.append(float(-450))
    arr0.append(float(e))
   #  if len(arr) == 0:
   #      arr.append(arr0)
    if arr0 not in arr:
        arr.append(arr0)

GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

def show_camera():
    # if video_capture.isOpened():
    if True:
       # ret_val, img = video_capture.read()
       img = cv2.imread('../images/test.png')

       imgContour = img.copy()
       imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
       imgBlur = cv2.GaussianBlur(imgGray,(7,7),1)
       imgCanny = cv2.Canny(imgBlur,50,50)
       
       data = []
       
       contours, hierarchy = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]
       
       for cnt in contours:
          area = cv2.contourArea(cnt)
          objectType = " "
          if area>400:
             #print(area)
             cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
             peri = cv2.arcLength(cnt,True)
             approx = cv2.approxPolyDP(cnt,0.02*peri,True)
             objCor = len(approx)
             #print("So dinh :", objCor)
             x, y, w, h = cv2.boundingRect(approx)
             if objCor == 3:
                objectType ="Triangle"
                add_pos(x,y,w,h,data,2)
                # print("this is a triangle")
                   
             elif objCor == 4:
                aspRatio = w/float(h)
                if aspRatio > 0.5 and aspRatio <1.5:
                   objectType= "Square"
                   add_pos(x,y,w,h,data,1)
                   # print("this is a square")
                   
             elif objCor>4:
                objectType= "Circles"
                add_pos(x,y,w,h,data,0)
                print("this is a Circles :", area, float(x+w//2), float(y+h//2))
             
             else:
                objectType="None"
                #print("So dinh :", objCor)
                   # a = float(x+w//2)
                   # b = float(y+h//2) 

             cv2.circle(imgContour,(x+w//2,y+h//2),5,(50,255,120),cv2.FILLED)
             cv2.circle(imgContour,(10,10),5,(55,255,120),cv2.FILLED)
             cv2.circle(imgContour,(1900,1000),5,(55,255,120),cv2.FILLED)
             cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),2)
             cv2.putText(imgContour,objectType,(x+(w//2)-10,y+(h//2)-10), cv2.FONT_HERSHEY_COMPLEX,0.7, (0,0,0),2)
       
       k = len(data)
       print(k,data)
       
       imgBlank = np.zeros_like(img)      
       imgStack = stackImages(0.8,([img,imgGray,imgBlur],[imgCanny,imgContour,imgBlank]))
       img_ss = cv2.resize(imgStack, (960, 540))

       while True:
         cv2.imshow("window_name", img_ss)
         if cv2.waitKey(1) & 0xFF == ord('q'):
           break
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
  # video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
  while True:
    show_camera()
      
