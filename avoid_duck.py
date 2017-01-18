#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect, BoolStamped, Twist2DStamped
import sys
import threading
from rgb_led import *
import numpy as np
import scipy as sp
import cv2

duck_detected = False

class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False

class Matcher:
    CONE = [np.array(x, np.uint8) for x in [[0,80,80], [22, 255,255]] ]
    DUCK = [np.array(x, np.uint8) for x in [[25,100,150], [35, 255, 255]] ]
    terms = {ObstacleType.CONE :"cone", ObstacleType.DUCKIE:"duck"}
    def __init__(self):
        self.cone_color_low = self.setupParam("~cone_low", [0,80,80])
        self.cone_color_high = self.setupParam("~cone_high", [22, 255,255])
        self.duckie_color_low = self.setupParam("~duckie_low", [25, 100, 150])
        self.duckie_color_high = self.setupParam("~duckie_high", [35, 255,255])
        self.CONE = [np.array(x, np.uint8) for x in [self.cone_color_low, self.cone_color_high] ]
        self.DUCK = [np.array(x, np.uint8) for x in [self.duckie_color_low, self.duckie_color_high] ]

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %('static_object_detector_node',param_name,value))
        return value

    def get_filtered_contours(self,img, contour_type):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        if contour_type == "CONE":
            frame_threshed = cv2.inRange(hsv_img, self.CONE[0], self.CONE[1])
            ret,thresh = cv2.threshold(frame_threshed,22,255,0)
        elif contour_type == "DUCK_COLOR":
            frame_threshed = cv2.inRange(hsv_img, self.DUCK[0], self.DUCK[1])
            ret,thresh = cv2.threshold(frame_threshed,30,255,0)
        elif contour_type == "DUCK_CANNY":
            frame_threshed = cv2.inRange(hsv_img, self.DUCK[0], self.DUCK[1])
            frame_threshed = cv2.adaptiveThreshold(frame_threshed,255,\
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)
            thresh = cv2.Canny(frame_threshed, 100,200)
        else:
            return 
        
        filtered_contours = []

        contours, hierarchy = cv2.findContours(\
                thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
        contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

        height,width = img.shape[:2]
        for (area,(cnt)) in contour_area:
        # plot box around contour
            x,y,w,h = cv2.boundingRect(cnt)
            box = (x,y,w,h)
            d =  0.5*(x-width/2)**2 + (y-height)**2 
            if not(h>15 and w >10 and h<200 and w<200 and d < 120000):
                continue
            if contour_type == "DUCK_CANNY":
                continue
            if contour_type =="DUCK_COLOR": # extra filtering to remove lines
                if not(h>25 and w>25):
                    continue
                if d>90000:
                    if not(h>35 and w>35):
                        continue
                if cv2.contourArea(cnt)==0:
                    continue
                val = cv2.arcLength(cnt,True)**2/ cv2.contourArea(cnt)
                if val > 35: continue
                rect = cv2.minAreaRect(cnt)
                ctr, sides, deg = rect
                val  = 0.5*cv2.arcLength(cnt,True) / (w**2+h**2)**0.5
                if val < 1.12: continue
                #if area > 1000: continue

            mask = np.zeros(thresh.shape,np.uint8)
            cv2.drawContours(mask,[cnt],0,255,-1)
            mean_val = cv2.mean(img,mask = mask)
            aspect_ratio = float(w)/h
            filtered_contours.append( (cnt, box, d, aspect_ratio, mean_val) )
        return filtered_contours


    def contour_match(self, img):
        print "4.  contour"
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''

        global duck_detected
        duck_detected = False

        object_list = ObstacleImageDetectionList()
        object_list.list = []

        height,width = img.shape[:2]
        object_list.imwidth = width
        object_list.imheight = height
        
        # get filtered contours
        cone_contours = self.get_filtered_contours(img, "CONE")
        duck_contours = self.get_filtered_contours(img, "DUCK_COLOR")

        all_contours = [duck_contours, cone_contours]
        for i, contours in enumerate(all_contours):
            for (cnt, box, ds, aspect_ratio, mean_color)  in contours:
                duck_detected = True
                # plot box around contour
                x,y,w,h = box
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(img,self.terms[i], (x,y), font, 0.5,mean_color,4)
                cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,2)
                
                r = Rect()
                r.x = x
                r.y = y
                r.w = w
                r.h = h

                t = ObstacleType()
                t.type = i

                d = ObstacleImageDetection()
                d.bounding_box = r
                d.type = t
                object_list.list.append(d);

        if duck_detected:
            print "duck is detected!!!"
            print "ATTEMPT TO OVERTAKE DUCK"
            #overtaking = "LSRSSRSLSS"
            #motion_planning(overtaking)

        return img, object_list

class StaticObjectDetectorNode:
    def __init__(self):
        self.name = 'static_object_detector_node'
        print "1.  static init"
        self.tm = Matcher()
        self.active = False
        self.thread_lock = threading.Lock()
        self.overtake = BoolStamped()
        self.overtake.data = False
        self.sub_image = rospy.Subscriber("~image_raw", Image, self.cbImage,queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch,queue_size=1)
        self.pub_image = rospy.Publisher("~cone_detection_image", Image, queue_size=1)
        self.pub_detections_list = rospy.Publisher("~detection_list", ObstacleImageDetectionList, queue_size=1)
        self.pub_overtake_over = rospy.Publisher("~overtake_done",BoolStamped, queue_size=1)
        self.bridge = CvBridge()
        turn_off_LEDs(speed=5)

        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)
        self.overtaking = "LSLBBBRBBRS"#fancy fancy overtake
        self.car_control_msg = Twist2DStamped()
        self.car_control_msg.v = 0
        self.car_control_msg.omega = 0
        self.moving = False

        rospy.loginfo("[%s] Initialized." %(self.name))

    def cbSwitch(self,switch_msg):
        self.active = switch_msg.data
        self.moving = False

    def cbImage(self,image_msg):
        if not self.active:
            print "not active"
            return
        print "2. cbImage"
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def car_command(self,v,omega,duration):
        self.car_control_msg.v = v
        self.car_control_msg.omega = omega
        self.pub_car_cmd.publish(self.car_control_msg)
        rospy.sleep(duration)
        self.car_control_msg.v = 0.0
        self.car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(self.car_control_msg)

    def motion_planning(self,concat):
        self.moving = True
        print "motion_planning"
        for i in range(len(concat)):
            primitives = concat[i]
            for case in switch(primitives):
                if case('S'):
                    self.car_command(0.5, 0, 0.6)
                    break
                if case('L'):
                    self.car_command(0.2, 4, 0.5)
                    break
                if case('R'):
                    self.car_command(0.2, -4, 0.5)
                    break
                if case('B'):
                    self.car_command(-0.5, 0, 0.4)
        self.overtake.data = True
        self.pub_overtake_over.publish(self.overtake)
    def processImage(self, image_msg):
        if self.moving:
            return
        print "3.  process image"
        if not self.thread_lock.acquire(True):
            print "3 process return"
            return
        try:
            image_cv=self.bridge.imgmsg_to_cv2(image_msg,"bgr8")
        except CvBridgeErrer as e:
            print e
        print "contour_match call"
        img, detections = self.tm.contour_match(image_cv)
        if duck_detected:
            print "duck_detected"
            self.motion_planning(self.overtaking)
        else:
            print "duck not detected"
        detections.header.stamp = image_msg.header.stamp
        detections.header.frame_id = image_msg.header.frame_id
        self.pub_detections_list.publish(detections)
        height,width = img.shape[:2]
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        self.thread_lock.release()

if __name__=="__main__":
	rospy.init_node('static_object_detector_node')
	node = StaticObjectDetectorNode()
	rospy.spin()
