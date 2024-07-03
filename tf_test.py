import yaml
import tf
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import rospy
from tf.transformations import euler_from_quaternion

def read_cam_external_parament(cam_ext_file_name):
        try:
            with open(cam_ext_file_name, "r", encoding="utf-8") as f:
                cam_extr = yaml.load(f, Loader=yaml.FullLoader)
                return cam_extr
        except:
            return None
def pose_to_mat44(camera_extr):

    # 四元数手眼矩阵转换为4*4齐次手眼转换矩阵
    trans_list = list(camera_extr['translation'].values())
    t = np.array(trans_list).reshape(1,3)
    x, y, z = trans_list[0],trans_list[1],trans_list[2]
    # print(t.shape)
    rota_list = list(camera_extr['rotation'].values())
    qx, qy, qz, qw = rota_list[0],rota_list[1],rota_list[2],rota_list[3]
    # T_eye_hand = tf.quaternion_matrix([qx, qy, qz, qw])
    T_eye_hand = R.from_quat([qx, qy, qz, qw]).as_matrix()
    # print(r.shape)
    # T_eye_hand = np.concatenate(
    #     (np.concatenate((R, t.T), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
    # T = T_eye_hand.as_matrix()
    # T_eye_hand[:3, 3] = [x, y, z]
    rotation = R.from_quat(list(camera_extr['rotation'].values())).as_matrix()
    translation = np.array(list(camera_extr['translation'].values())).reshape(3, 1)
    mat44 = np.concatenate([np.concatenate([rotation, np.zeros([1, 3])], axis=0), np.concatenate([translation, np.ones([1, 1])], axis=0)],axis=1)

    return mat44
    # return T

def get_cam2end(cam_ext_file_name):
     camera_extr = read_cam_external_parament(cam_ext_file_name)
     mat = pose_to_mat44(camera_extr)
     return mat
    
def tcp_transform_base(grasp2end):
    rospy.init_node('tf_listener', anonymous=True)
    listener = tf.TransformListener()
    sleep(0.6)
    translation,rotation = listener.lookupTransform('base', 'tool0', rospy.Time(0))
    mat44 = listener.fromTranslationRotation(translation, rotation)
    # print(mat44)
    grasp = np.dot(mat44, grasp2end)
    return grasp

def get_robot_pose():
    rospy.init_node('tf_listener', anonymous=True)
    listener = tf.TransformListener()
    sleep(0.6)
    translation,rotation = listener.lookupTransform('base', 'tool0', rospy.Time(0))
    q = [rotation[3],rotation[0],rotation[1],rotation[2]]
    rqt = euler_from_quaternion(q)
    print(rqt)
    print(translation)
    return 0

if __name__ =="__main__":
    # cam_pose_calibration_file = "real/cam_pose/calibration.yaml"
    # cam_pose_calibration = get_cam2end(cam_pose_calibration_file)
    # print(cam_pose_calibration)
    get_robot_pose()