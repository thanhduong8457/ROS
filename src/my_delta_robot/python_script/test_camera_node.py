#!/usr/bin/env python
import rospy
import cv2

from std_msgs.msg import String
from my_delta_robot.msg import image_pos

GSTREAMER_PIPELINE = 'nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1920, height=1080, format=(string)NV12, framerate=10/1 ! nvvidconv flip-method=0 ! video/x-raw, width=960, height=616, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink wait-on-eos=false max-buffers=1 drop=True'

m = 1.0
k1 = 0.0
k2 = 0.0

def show_camera():
    global is_continue_to_take_another_image
    ######  Home Node  ######
    rospy.init_node("camera_node", anonymous=False)
    rate = rospy.Rate(7.8125)  # Hz

    #######  Subscriber  ##################
    rospy.Subscriber("status_to_image_node", String, image_node_callback) # Topics

    #######  Publisher  ##################
    data_image_pub = rospy.Publisher("data_image", image_pos, queue_size=10)

    data_image = image_pos()
    is_continue_to_take_another_image = True

    while not rospy.is_shutdown():
    # if video_capture.isOpened():
        if (is_continue_to_take_another_image):
            # ret_val, img = video_capture.read()
            img = cv2.imread('../images/test.png')

            imgContour = img.copy()
            imgGray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            imgBlur = cv2.GaussianBlur(imgGray,(7,7),1)
            imgCanny = cv2.Canny(imgBlur,50,50)
            
            x_obj = []
            y_obj = []
            z_obj = []
            type_obj = []
            
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
                        add_pos(x, y, w, h, x_obj, y_obj, z_obj, type_obj, 2)
                        # print("this is a triangle")
                        
                    elif objCor == 4:
                        aspRatio = w/float(h)
                        if aspRatio > 0.5 and aspRatio <1.5:
                            objectType= "Square"
                            add_pos(x,y,w,h, x_obj, y_obj, z_obj, type_obj, 1)
                            # print("this is a square")
                        
                    elif objCor>4:
                        objectType= "Circles"
                        add_pos(x,y,w,h, x_obj, y_obj, z_obj, type_obj, 0)
                        # print("this is a Circles :", area, float(x+w//2), float(y+h//2))
                    
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
            
            print(len(x_obj), x_obj, y_obj, z_obj, type_obj)

            is_wait = False
            if(len(x_obj)==0):
                is_wait = True
            else:
                # Send data thought topic 
                data_image.x = x_obj
                data_image.y = y_obj
                data_image.z = z_obj
                data_image.type = type_obj
                rospy.sleep(1)
                data_image_pub.publish(data_image)

            is_continue_to_take_another_image = False

        else:
            if(is_wait):
                print("sleep for 5s and process another images")
                rospy.sleep(5)
                is_continue_to_take_another_image = True
                is_wait = False

            rate.sleep()

    return
    
def add_pos(a, b, c, d, x, y, z, type, e):
    x_temp = 0.4585*(float(a+c//2)*m-k1)-212.2
    y_temp = 237.7-0.3897*(float(b+d//2))

    # if((x_temp**2 + y_temp**2) >= 158):
    #     return
        
    x.append(x_temp) 
    y.append(y_temp)
    z.append(float(-450))
    type.append(int(e))

    # if arr0 not in arr:
    #     arr.append(arr0)

def image_node_callback(data):
    global is_continue_to_take_another_image
    print("Status from node_a: ", data)
    is_continue_to_take_another_image = True

if __name__ == "__main__":
    try:
        # video_capture = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
        show_camera()
    except rospy.ROSInterruptException:
        pass
