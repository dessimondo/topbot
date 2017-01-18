#!/usr/bin/env python
import rospkg
import rospy
import yaml
import math
from duckietown_msgs.msg import AprilTagsWithInfos, TagInfo, AprilTagDetectionArray, Twist2DStamped, BoolStamped
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class AprilPostPros(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = "apriltags_postprocessing_node"

        # Load parameters
        self.camera_x     = self.setupParam("~camera_x", 0.065)
        self.camera_y     = self.setupParam("~camera_y", 0.0)
        self.camera_z     = self.setupParam("~camera_z", 0.11)
        self.camera_theta = self.setupParam("~camera_theta", 19.0)
        self.scale_x     = self.setupParam("~scale_x", 1)
        self.scale_y     = self.setupParam("~scale_y", 1)
        self.scale_z     = self.setupParam("~scale_z", 1)
        self.apriltag_detected = BoolStamped()
        self.apriltag_detected.data = False
        self.duck_detected = BoolStamped()
        self.active = False
        self.no_detect_count = 0

# -------- Start adding back the tag info stuff

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltags_ros')
        tags_filepath = self.setupParam("~tags_file", self.pkg_path+"/apriltagsDB/apriltagsDB.yaml") # No tags_file input atm., so default value is used
        self.loc = self.setupParam("~loc", -1) # -1 if no location is given
        tags_file = open(tags_filepath, 'r')
        self.tags_dict = yaml.load(tags_file)
        tags_file.close()
        self.info = TagInfo()

        
# ---- end tag info stuff

        self.sub_prePros = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped,self.cbSwitch,queue_size=1)
        self.pub_postPros = rospy.Publisher("~apriltags_out", AprilTagsWithInfos, queue_size=1)
        self.pub_visualize = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_duck_detected = rospy.Publisher("~duck_detected", BoolStamped, queue_size=1)
        self.pub_apriltag_detection = rospy.Publisher("~detection",BoolStamped,queue_size=1)
        rospy.loginfo("[%s] has started", self.node_name)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbSwitch(self,msg):
        self.active = msg.data
    def callback(self, msg):
        #print self.apriltag_detected.data
        if not self.active:
            #print "postprocess not active"
            return
        #print "postprocess active"
        tag_infos = []
        self.duck_detected.data = False
        car_cmd_msg = Twist2DStamped()
        #detections length
        detected = False
        #print "time ",rospy.Time.now().secs
        # Load tag detections message
        for detection in msg.detections:
            self.no_detect_count = 0
            self.apriltag_detected.data =False
            self.pub_apriltag_detection.publish(self.apriltag_detected)
            detected = True
            # ------ start tag info processing

            new_info = TagInfo()
            new_info.id = int(detection.id)
            id_info = self.tags_dict[new_info.id]
            
            # Define the transforms
            veh_t_camxout = tr.translation_matrix((self.camera_x, self.camera_y, self.camera_z))
            veh_R_camxout = tr.euler_matrix(0, self.camera_theta*np.pi/180, 0, 'rxyz')
            veh_T_camxout = tr.concatenate_matrices(veh_t_camxout, veh_R_camxout)   # 4x4 Homogeneous Transform Matrix

            camxout_T_camzout = tr.euler_matrix(-np.pi/2,0,-np.pi/2,'rzyx')
            veh_T_camzout = tr.concatenate_matrices(veh_T_camxout, camxout_T_camzout)

            tagzout_T_tagxout = tr.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')

            #Load translation
            trans = detection.pose.pose.position
            rot = detection.pose.pose.orientation

            camzout_t_tagzout = tr.translation_matrix((trans.x*self.scale_x, trans.y*self.scale_y, trans.z*self.scale_z))
            camzout_R_tagzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
            camzout_T_tagzout = tr.concatenate_matrices(camzout_t_tagzout, camzout_R_tagzout)

            veh_T_tagxout = tr.concatenate_matrices(veh_T_camzout, camzout_T_tagzout, tagzout_T_tagxout)

            # Overwrite transformed value
            (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagxout)
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagxout)

            detection.pose.pose.position = trans
            detection.pose.pose.orientation = rot

            print "detected tag ID: %d" % new_info.id
            #get the distance
            distance = math.sqrt(math.pow(trans.x,2)+math.pow(trans.y,2))
            #print "distance %f" % distance
            ratio = distance/0.8
            #get the angle
            theta = math.atan2(trans.y, trans.x)
            if math.fabs(theta)<0.174:#10 degrees min turning degree
                theta = 0
            elif theta>0.69:#40 degrees max turning degree 
                theta = 0.69
            elif theta<-0.69:
                theta = -0.69
            degree = theta*180/math.pi
            #print "degree %f" % degree
            # Check yaml file to fill in ID-specific information
            #new_info.tag_type = self.sign_types[id_info['tag_type']]

            if new_info.id == 6:
                if distance<0.3:
                    car_cmd_msg.v=0
                    car_cmd_msg.omega=0
                    self.pub_car_cmd.publish(car_cmd_msg)
                    self.duck_detected.data = True
                    self.pub_duck_detected.publish(self.duck_detected)
                else:
                    car_cmd_msg.v = 0.3
                    car_cmd_msg.omega = theta*2
                    self.pub_car_cmd.publish(car_cmd_msg)
                break
            elif new_info.id == 4:
                if distance>0.3:
                    if theta == 0:#go strait
                        print "Go Strait"
                        car_cmd_msg.v = 0.6*ratio
                        car_cmd_msg.omega = 0
                        self.pub_car_cmd.publish(car_cmd_msg)
                    else:
                        print "turn"
                        car_cmd_msg.v = 0.5*ratio
                        car_cmd_msg.omega = 3.3*theta
                        self.pub_car_cmd.publish(car_cmd_msg)
                        rospy.sleep(0.3)
                        car_cmd_msg.v = 0.4*ratio
                        car_cmd_msg.omega = 0
                        self.pub_car_cmd.publish(car_cmd_msg)
                    #rospy.sleep(0.3)
                    #print "CAR SHOULD MOVE"
                    break
                elif distance<=0.3 and math.fabs(theta)==0.69:
                    print "close and large degree"
                    theta = math.atan2(trans.y, trans.x)
                    car_cmd_msg.v = 0.15
                    car_cmd_msg.omega = 3.5*theta
                    self.pub_car_cmd.publish(car_cmd_msg)
                    rospy.sleep(0.4)
                    car_cmd_msg.v = 0
                    car_cmd_msg.omega = 0
                    self.pub_car_cmd.publish(car_cmd_msg)
                    break
                else:
                    car_cmd_msg.v=0
                    car_cmd_msg.omega=0
                    self.pub_car_cmd.publish(car_cmd_msg)
                    print "too close"
                    break
            elif new_info.id == 5:
                if distance>0.3:
                    if theta == 0:#go strait
                        print "Go Strait"
                        car_cmd_msg.v = 0.6*ratio
                        car_cmd_msg.omega = 0
                        self.pub_car_cmd.publish(car_cmd_msg)
                    else:
                        print "turn"
                        car_cmd_msg.v = 0.5*ratio
                        car_cmd_msg.omega = 3.3*theta
                        self.pub_car_cmd.publish(car_cmd_msg)
                        rospy.sleep(0.3)
                        car_cmd_msg.v = 0.4*ratio
                        car_cmd_msg.omega = 0
                        self.pub_car_cmd.publish(car_cmd_msg)
                     #rospy.sleep(0.3)
                     #print "CAR SHOULD MOVE"
                    break
                elif distance<=0.3 and math.fabs(theta)==0.69:
                    print "close and large degree"
                    theta = math.atan2(trans.y, trans.x)
                    car_cmd_msg.v = 0.15
                    car_cmd_msg.omega = 3.5*theta
                    self.pub_car_cmd.publish(car_cmd_msg)
                    rospy.sleep(0.4)
                    car_cmd_msg.v = 0
                    car_cmd_msg.omega = 0
                    self.pub_car_cmd.publish(car_cmd_msg)
                    break
                else:
                    car_cmd_msg.v=0
                    car_cmd_msg.omega=0
                    self.pub_car_cmd.publish(car_cmd_msg)
                    print "too close"
                    break
                
            tag_infos.append(new_info)
            # --- end tag info processing
        
		#if no detections
        if detected == False:
            self.no_detect_count +=1
            print "no_detect_count %d" % self.no_detect_count
            car_cmd_msg.v = 0
            car_cmd_msg.omega = 0
            self.pub_car_cmd.publish(car_cmd_msg)
            rospy.sleep(0.5)
            print "no detections"
            if self.no_detect_count >= 50:
                print "send alert"
                self.apriltag_detected.data = True
                self.pub_apriltag_detection.publish(self.apriltag_detected)

        new_tag_data = AprilTagsWithInfos()
        new_tag_data.detections = msg.detections
        new_tag_data.infos = tag_infos
        # Publish Message
        self.pub_postPros.publish(new_tag_data)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
