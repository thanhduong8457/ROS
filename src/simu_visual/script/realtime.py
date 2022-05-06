#!/usr/bin/env python\
import cv2
import numpy as np

m = 1.0
k1 = 0.0
k2 = 0.0

# Khung hinh doc ve (a, b) pixel ==> 1 (pixel) se tuong duong voi m (mm)
# Gia su vi tri doc ve tu camera la A(x, y) se tuong duong voi A(x.m, y.m) (mm)
# Diem O1 tam cua camera doi voi he truc toa do O2 co vi tri O1(k1,k2)
# Vi tri diem A can tim se la A(x.m - k1, y.m - k2)

def add_pos(a,b,c,d,arr):
    arr0 = []
    arr0.append(float(a+c//2)*m-k1)
    arr0.append(float(b+d//2)*m-k2)
    arr0.append(float(-400))
    if len(arr) == 0:
        arr.append(arr0)
    # if arr0 not in arr:
    #     arr.append(arr0)

def node():
   #######  callback_linear_speed_xyz(data)  #######
   global call_xo
   global call_yo
   global call_zo
   
   path = '../images/images_2.jpg'

   img = cv2.imread(path)
   imgContour = img.copy()
   imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
   imgBlur = cv2.GaussianBlur(imgGray,(7,7),1)
   imgCanny = cv2.Canny(imgBlur,50,50)

   triangle = []
   square = []
   rectang = []
   circles = [] 

   contours, hierarchy = cv2.findContours(imgCanny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]

   for cnt in contours:
      area = cv2.contourArea(cnt)
      if area>400:
         cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3)
         peri = cv2.arcLength(cnt,True)
         approx = cv2.approxPolyDP(cnt,0.02*peri,True)
         objCor = len(approx)
         x, y, w, h = cv2.boundingRect(approx)

         if objCor == 3:
            objectType ="Triangle"
            add_pos(x,y,w,h,triangle)
            print("this is a triangle")
               
         elif objCor == 4:
            aspRatio = w/float(h)
            if aspRatio >0.98 and aspRatio <1.03:
               objectType= "Square"
               add_pos(x,y,w,h,square)
               print("this is a square")
               
            else:
               objectType="Rectangle"
               add_pos(x,y,w,h,rectang)
               print("this is a Rectangle")
               
         elif objCor>4:
            objectType= "Circles"
            add_pos(x,y,w,h,circles)
            print("this is a Circles")
         
         else:
            objectType="None"
               # a = float(x+w//2)
               # b = float(y+h//2) 
               # print("Center :[",a,",",b,"]")

         cv2.circle(imgContour,(x+w//2,y+h//2),5,(50,255,120),cv2.FILLED)
         cv2.circle(imgContour,(10,10),5,(55,255,120),cv2.FILLED)
         cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),2)

         cv2.putText(imgContour,objectType,(x+(w//2)-10,y+(h//2)-10), cv2.FONT_HERSHEY_COMPLEX,0.7, (0,0,0),2)

      
   print(triangle)
   print(square)
   print(rectang)
   print(circles)

   cv2.imshow("window_name", imgContour)
   cv2.waitKey(0)
   cv2.destroyAllWindows() 

   return

if __name__ == "__main__":
   node()
