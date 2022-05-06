#!/usr/bin/env python
import numpy as np

import rospy
import cv2

from simu_visual.msg import linear_speed_xyz
from simu_visual.msg import PositionArm

m = 1.0
k1 = 0.0
k2 = 0.0

request_triangle = 0
request_square = 0
request_rectang = 0
request_circles = 0

response_triangle = 0
response_square = 0
response_rectang = 0
response_circles = 0

# Khung hinh doc ve (a, b) pixel ==> 1 (pixel) se tuong duong voi m (mm)
# Gia su vi tri doc ve tu camera la A(x, y) se tuong duong voi A(x.m, y.m) (mm)
# Diem O1 tam cua camera doi voi he truc toa do O2 co vi tri O1(k1,k2)
# Vi tri diem A can tim se la A(x.m - k1, y.m - k2)

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

def add_pos(a,b,c,d,arr):
    arr0 = []
    arr0.append(float(a+c//2)*m-k1)
    arr0.append(float(b+d//2)*m-k2)
    arr0.append(float(-400))
    if len(arr) == 0:
        arr.append(arr0)
    # if arr0 not in arr:
    #     arr.append(arr0)

def check_pos_valid(arr_1,arr_2):
    for item in arr_1 :
        if item not in arr_2 :
            arr_1.remove(item)

def node():
   #######  callback_linear_speed_xyz(data)  #######
   global call_xo
   global call_yo
   global call_zo
   
   path = '/home/manhduc1210/catkin_ws/src/simu_visual/Tracking_Project/Resources/img2.jpg'

   img = cv2.imread(path)
   imgContour = img.copy()
   imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
   imgBlur = cv2.GaussianBlur(imgGray,(7,7),1)
   imgCanny = cv2.Canny(imgBlur,50,50)

   triangle = []
   square = []
   rectang = []
   circles = [] 
   
   pos_triangle = [0.0, 0.0, -200.0]
   pos_square = [0.0, 0.0, -200.0]
   pos_rectang = [0.0, 0.0, -200.0]
   pos_circles = [0.0, 0.0, -200.0]
   
   home = [0.0, 0.0, -250.0]

   realtime_permiso = True
   realtime_id_call = 1
   realtime_id_permiso = 0
   realtime_vmax = 2000.0
   realtime_amax = 40000.0
   realtime_paso1 = 50.0
   realtime_paso2 = 150.0
   
##
   realtime_pos = linear_speed_xyz()
   #######  Variables verify incoming message  #######
   call_xo = 0.0
   call_yo = 0.0
   call_zo = 0.0
   
   realtime_xf = 0.0
   realtime_yf = 0.0
   realtime_zf = 0.0

   temp = 0
   temp1 = 0
   temp2 = 0
   temp3 = 0
   temp4 = 0
   
   #######  Start ROS node  ##################
   rospy.init_node("realtime_auto", anonymous=False)
   rate = rospy.Rate(5.0)  # Hz

   #######  Publisher  ##################
   pub1 = rospy.Publisher("input_ls_final", linear_speed_xyz, queue_size=10)
   rospy.Subscriber("position_callback", PositionArm, callbackpos)
   
   while not rospy.is_shutdown():
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
               if len(triangle) != 0:
                  request_triangle = 1
                  response_triangle = 0
               else:
                  request_triangle = 0
                  response_triangle = 1                  
            elif objCor == 4:
               aspRatio = w/float(h)
               if aspRatio >0.98 and aspRatio <1.03:
                  objectType= "Square"
                  add_pos(x,y,w,h,square)
                  if len(square) != 0:
                     request_square = 1
                     response_square = 0
                  else:
                     request_square = 0
                     response_square = 1                  
               else:
                  objectType="Rectangle"
                  add_pos(x,y,w,h,rectang)
                  if len(rectang) != 0:
                     request_rectang = 1
                     response_rectang = 0
                  else:
                     request_rectang = 0
                     response_rectang = 1                  
            elif objCor>4:
               objectType= "Circles"
               add_pos(x,y,w,h,circles)
               if len(circles) != 0:
                  request_circles = 1
                  response_circles = 0
               else:
                  request_circles = 0
                  response_circles = 1                  
            else:
               objectType="None"
                # a = float(x+w//2)
                # b = float(y+h//2) 
                # print("Center :[",a,",",b,"]")
            cv2.circle(imgContour,(x+w//2,y+h//2),5,(50,255,120),cv2.FILLED)
            cv2.circle(imgContour,(10,10),5,(55,255,120),cv2.FILLED)
            cv2.rectangle(imgContour,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(imgContour,objectType,(x+(w//2)-10,y+(h//2)-10), cv2.FONT_HERSHEY_COMPLEX,0.7, (0,0,0),2)

      if(temp == 0):
         if(request_triangle == 1):
            if(temp1 == 0):
               realtime_xf = triangle[0][0]
               realtime_yf = triangle[0][1]
               realtime_zf = triangle[0][2]
               realtime_id_call += 1
               temp1 += 1
            if((temp1 == 1) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_xf = pos_triangle[0]
               realtime_yf = pos_triangle[1]
               realtime_zf = pos_triangle[2]
               realtime_id_call += 1
               temp1 += 1
            if((temp1 == 2) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_id_call += 1
               request_triangle = 0
               response_triangle = 1
               temp1 = 0
               del triangle[0]
               temp += 1
         else:
               temp += 1
      elif(temp == 1):
         if(request_square == 1):
            if(temp2 == 0):
               realtime_xf = square[0][0]
               realtime_yf = square[0][1]
               realtime_zf = square[0][2]
               realtime_id_call += 1
               temp2 += 1
            if((temp2 == 1) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_xf = pos_square[0]
               realtime_yf = pos_square[1]
               realtime_zf = pos_square[2]
               realtime_id_call += 1
               temp2 += 1
            if((temp2 == 2) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_id_call += 1
               request_square = 0
               response_square = 1
               temp2 = 0
               del square[0]
               temp += 1
         else:
               temp += 1
      elif(temp == 2):
         if(request_rectang == 1):
            if(temp3 == 0):
               realtime_xf = rectang[0][0]
               realtime_yf = rectang[0][1]
               realtime_zf = rectang[0][2]
               realtime_id_call += 1
               temp3 += 1
            if((temp3 == 1) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_xf = pos_rectang[0]
               realtime_yf = pos_rectang[1]
               realtime_zf = pos_rectang[2]
               realtime_id_call += 1
               temp3 += 1
            if((temp3 == 2) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_id_call += 1
               request_rectang = 0
               response_rectang = 1
               temp3 = 0
               del rectang[0]
               temp += 1
         else:
               temp += 1
      elif(temp == 3):
         if(request_circles == 1):
            if(temp4 == 0):
               realtime_xf = circles[0][0]
               realtime_yf = circles[0][1]
               realtime_zf = circles[0][2]
               realtime_id_call += 1
               temp4 += 1
            if((temp4 == 1) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_xf = pos_circles[0]
               realtime_yf = pos_circles[1]
               realtime_zf = pos_circles[2]
               realtime_id_call += 1
               temp4 += 1
            if((temp4 == 2) and (call_xo == realtime_xf) and (call_yo == realtime_yf) and (call_yo == realtime_yf)):
               realtime_id_call += 1
               request_circles = 0
               response_circles = 1
               temp4 = 0
               del circles[0]
               temp += 1
         else:
               temp += 1
      else:
            temp = 0
            
      realtime_permiso = True
      realtime_pos.xo = call_xo
      realtime_pos.yo = call_yo
      realtime_pos.zo = call_zo
      realtime_pos.xf = realtime_xf
      realtime_pos.yf = realtime_yf
      realtime_pos.zf = realtime_zf
      realtime_pos.vmax = realtime_vmax
      realtime_pos.amax = realtime_amax
      realtime_pos.paso1 = realtime_paso1
      realtime_pos.paso2 = realtime_paso2
      realtime_pos.ls_fin = realtime_permiso
      realtime_pos.idcall = realtime_id_call
      realtime_pos.num_tray = 1

      print("xo,yo,zo :",realtime_pos.xo,realtime_pos.yo,realtime_pos.zo)
      print("xf,yf,zf :",realtime_pos.xf,realtime_pos.yf,realtime_pos.zf,realtime_id_call)
      print("triangle :",triangle)
      print("request_triangle :",request_triangle)

      pub1.publish(realtime_pos)      
      rate.sleep()
   return

def callbackpos(data):
   global call_xo
   global call_yo
   global call_zo
   
   call_xo = data.joint_10
   call_yo = data.joint_11
   call_zo = data.joint_12
   
if __name__ == "__main__":
   try:
      node()
   except rospy.ROSInterruptException:
      pass
