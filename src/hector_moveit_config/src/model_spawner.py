#!/usr/bin/env python 
import rospy
from gazebo_msgs.srv import *
from geometry_msgs.msg import *

def run():
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)
        model_name = 'apple'
        f = open('/home/tahsincan/.gazebo/models/apple/model.sdf','r')
        model_xml = f.read()
        pose = Pose()
        pose.position.z = 10
        pose.orientation.w = 1 
        response = spawn_model(model_name,model_xml,"",pose,"")
        print response.status_message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
if __name__ == '__main__':
    run()