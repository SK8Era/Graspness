# import socket
# HOST = "192.168.1.10"    # The remote host
# PORT = 30003        # The same port as used by the server
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((HOST, PORT))
 
# strL = b"movel(p[-0.02,-0.08,0.3,1,-0.1,0.1],a=0.5,v=0.3)\n"
# s.send(strL)
 
# s.close()
import sys
# sys.path.append('../')
sys.path.append("/home/jiyuan/tir_ws/src")
from ur_planning.srv import grasp_pose,grasp_poseRequest,grasp_poseResponse

# moveit_server = moveitServer.MoveIt_Control(is_use_gripper=False)