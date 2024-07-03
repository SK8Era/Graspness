import os
import sys
import numpy as np
import argparse
from PIL import Image
import time
import scipy.io as scio
import torch
import open3d as o3d
from UR_Robot import UR_Robot
from real.realsenseD415 import Camera
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
from data_utils import CameraInfo, create_point_cloud_from_depth_image, get_workspace_mask
parser = argparse.ArgumentParser()
parser.add_argument('--num_point', type=int, default=15000, help='Point Number [default: 15000]')
cfgs = parser.parse_args()

rs_cam = Camera(width=1280, height=720)

intrinsic = np.array([615.284,0,309.623,0,614.557,247.967,0,0,1]).reshape(3,3)
cam_depth_scale = np.loadtxt('real/cam_pose/camera_depth_scale.txt', delimiter=' ')
factor_depth = 1.0 / cam_depth_scale
save_pointcloud_name = "result/pc_yenai.pcd"
rgb, depth = rs_cam.get_data()
color = rgb / 255
color = color[:, :, ::-1]
depth = depth.astype(np.float32)
workspace_mask = np.array(Image.open(os.path.join('doc/example_data', 'workspace_mask.png'))) #[720,1280][241false,978false]
camera = CameraInfo(1280.0, 720.0, intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2],
                    factor_depth)
# compute workspace limits
x_left_up,y_left_up = 0+25,242+25
x_right_bottom,y_right_bottom = 719-25,977-25
point_z = depth[x_left_up,y_left_up] / camera.scale
point_x = (y_left_up - camera.cx) * point_z / camera.fx
point_y = (x_left_up - camera.cy) * point_z / camera.fy
point_left_up = (point_x,point_y,point_z)
point_z = depth[x_right_bottom,y_right_bottom] / camera.scale
point_x = (y_right_bottom - camera.cx) * point_z / camera.fx
point_y = (x_right_bottom - camera.cy) * point_z / camera.fy
point_right_bottom = (point_x, point_y, point_z)

# generate cloud
cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)#720,1080,3

# get valid points
depth_mask = (depth > 0)
# camera_poses = np.load(os.path.join(root, 'scenes', scene_id, camera_type, 'camera_poses.npy'))
# align_mat = np.load(os.path.join(root, 'scenes', scene_id, camera_type, 'cam0_wrt_table.npy'))
# trans = np.dot(align_mat, camera_poses[int(index)])
# workspace_mask = get_workspace_mask(cloud, seg, trans=trans, organized=True, outlier=0.02)
mask = (workspace_mask & depth_mask)

cloud_masked = cloud[mask]#51225,3
color_masked = color[mask]
# sample points random
if len(cloud_masked) >= cfgs.num_point:
    idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
else:
    idxs1 = np.arange(len(cloud_masked))
    idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point - len(cloud_masked), replace=True)
    idxs = np.concatenate([idxs1, idxs2], axis=0)
cloud_sampled = cloud_masked[idxs]#15000,3
color_sampled = color_masked[idxs]

cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32)) #51w points
o3d.io.write_point_cloud(save_pointcloud_name, cloud, write_ascii=False, compressed=False, print_progress=False)
o3d.visualization.draw_geometries([cloud])