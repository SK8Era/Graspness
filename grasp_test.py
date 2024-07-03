#!/usr/bin/env python
# -*-coding:utf8 -**-
import sys
sys.path.append("/home/jiyuan/tir_ws/devel_isolated/ur_planning/include")
sys.path.append("/home/jiyuan/tir_ws/src")
import rospy
from ur_planning.srv import grasp_pose,grasp_poseRequest,grasp_poseResponse

if __name__=="__main__":
    rospy.init_node("g_test_client")
    client = rospy.ServiceProxy("moveit_grasp", grasp_pose)
    rospy.loginfo("数据返回发送")
    response = client.call(0.0,0.0,0.0,0.0,0.0,0.0)
    rospy.loginfo(response)
    # rospy.spin()