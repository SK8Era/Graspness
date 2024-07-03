import urx
import time
import math
import socket
import struct
import numpy as np
def main():
    # 替换为你的UR机械臂的IP地址
    robot_ip = "192.168.1.60"

    # 连接到UR机械臂
    rob = urx.Robot(robot_ip)
    
    try:
        while True:
            # 获取当前的TCP位置
            tcp_pose = rob.getl()
            # tcp_pose 返回一个包含6个元素的列表：[x, y, z, rx, ry, rz]
            # 其中x, y, z是位置，rx, ry, rz是旋转（轴角）

            # 将旋转部分（轴角）转换为RPY
            rpy = rob.get_pose().orientation.rpy()

            # 打印位姿
            print(f"Position: X={tcp_pose[0]:.4f}, Y={tcp_pose[1]:.4f}, Z={tcp_pose[2]:.4f}")
            print(f"Orientation (RPY): R={rpy[0]:.4f}, P={rpy[1]:.4f}, Y={rpy[2]:.4f}")
            
            # 每秒获取一次
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # 关闭与UR机械臂的连接
        rob.close()

def get_position(tcp_host_ip,tcp_port):
    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.connect((tcp_host_ip, tcp_port))
    state_data = tcp_socket.recv(1500)
    actual_tool_positions = parse_tcp_state_data(state_data, 'cartesian_info')
    print(actual_tool_positions)
    tcp_socket.close()
    return state_data

def parse_tcp_state_data(data, subpasckage):
    dic = {'MessageSize': 'i', 'Time': 'd', 'q target': '6d', 'qd target': '6d', 'qdd target': '6d',
            'I target': '6d',
            'M target': '6d', 'q actual': '6d', 'qd actual': '6d', 'I actual': '6d', 'I control': '6d',
            'Tool vector actual': '6d', 'TCP speed actual': '6d', 'TCP force': '6d', 'Tool vector target': '6d',
            'TCP speed target': '6d', 'Digital input bits': 'd', 'Motor temperatures': '6d', 'Controller Timer': 'd',
            'Test value': 'd', 'Robot Mode': 'd', 'Joint Modes': '6d', 'Safety Mode': 'd', 'empty1': '6d',
            'Tool Accelerometer values': '3d',
            'empty2': '6d', 'Speed scaling': 'd', 'Linear momentum norm': 'd', 'SoftwareOnly': 'd',
            'softwareOnly2': 'd',
            'V main': 'd',
            'V robot': 'd', 'I robot': 'd', 'V actual': '6d', 'Digital outputs': 'd', 'Program state': 'd',
            'Elbow position': 'd', 'Elbow velocity': '3d'}
    ii = range(len(dic))
    for key, i in zip(dic, ii):
        fmtsize = struct.calcsize(dic[key])
        data1, data = data[0:fmtsize], data[fmtsize:]
        fmt = "!" + dic[key]
        dic[key] = dic[key], struct.unpack(fmt, data1)

    if subpasckage == 'joint_data':  # get joint data
        q_actual_tuple = dic["q actual"]
        joint_data= np.array(q_actual_tuple[1])
        return joint_data
    elif subpasckage == 'cartesian_info':
        Tool_vector_actual = dic["Tool vector actual"]  # get x y z rx ry rz
        cartesian_info = np.array(Tool_vector_actual[1])
        return cartesian_info
    
if __name__ == "__main__":
    main()
