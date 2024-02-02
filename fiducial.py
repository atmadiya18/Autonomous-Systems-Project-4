#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransformArray
import tf2_ros
from math import pi
import math
import time

def degrees(r):
    return (180*r/math.pi)

class Fiducial:
    # defining constructor for the class
    def __init__(self):
        rospy.init_node('Fiducial')

        #Defining publisher for motion 
        
        self.pub_m = rospy.Publisher("/cmd_vel",Twist,queue_size=10)

        #Defining the fiducial 

        self.chosen_fid = rospy.get_param("~target_fiducial", "fid108")

        #Defining the distance 

        self.dist = rospy.get_param("distance", 1.5)

        #Subscriber

        rospy.Subscriber ("/fiducial_transforms", FiducialTransformArray, self.uctronics)
        self.x = self.dist
        self.y = 0

    def uctronics(self,msg):
        for i in msg.transforms:
            id = i.fiducial_id
            trans = i.transform.translation
            rot = i.transform.rotation
            print ("Fid %d %lf %lf %lf %lf %lf %lf %lf\n" % \
                                 (id, trans.x, trans.y, trans.z,
                                  rot.x, rot.y, rot.z, rot.w))


            mat = TransformStamped()
            mat.transform.translation.x = trans.x
            mat.transform.translation.y = trans.y
            mat.transform.translation.z = trans.z
            mat.transform.rotation.x = rot.x
            mat.transform.rotation.y = rot.y
            mat.transform.rotation.z = rot.z
            mat.transform.rotation.w = rot.w
            self.br.sendTransform(mat)

    def move(self):

        #Setting the rate 
        rate = rospy.Rate(10)

        #While node is running 
        while not rospy.is_shutdown():
            vel_msg = motors()
            rate.sleep()
if __name__ == "__main__":
    node = Fiducial()
    node.move()
