from graspnetAPI import GraspNet
import os
import open3d as o3d
import numpy as np


def vis(dataset_root, obj_idx, num_grasp=10, th=0.5, max_width=0.08, save_folder='save_fig', show=False):
    plyfile = os.path.join(dataset_root, 'models', '%03d'%obj_idx, 'nontextured.ply')
    model = o3d.io.read_point_cloud(plyfile)

    num_views, num_angles, num_depths = 300, 12, 4
    views = generate_views(num_views)

    vis = o3d.visualization.Visualizer()
    vis.create_window(width = 1280, height = 720)
    ctr = vis.get_view_control()
    param = get_camera_parameters(camera='relasense')

    # cam_pos = np.load(os.path.join(dataset_root, 'scenes', 'scene_0000', 'kinect', 'cam0_wrt_table.npy'))
    cam_pos = np.load(os.path.join(dataset_root, 'cam0_wrt_table.npy'))
    param.extrinsic = np.linalg.inv(cam_pos).tolist()

    sampled_points, offsets, scores, _ = get_model_grasps('%s/grasp_label/%03d_labels.npz'%(dataset_root, obj_idx))

    cnt = 0
    point_inds = np.arange(sampled_points.shape[0])
    np.random.shuffle(point_inds)
    grippers = []

    for point_ind in point_inds:
        target_point = sampled_points[point_ind]
        offset = offsets[point_ind]
        score = scores[point_ind]
        view_inds = np.arange(300)
        np.random.shuffle(view_inds)
        flag = False
        for v in view_inds:
            if flag: break
            view = views[v]
            angle_inds = np.arange(12)
            np.random.shuffle(angle_inds)
            for a in angle_inds:
                if flag: break
                depth_inds = np.arange(4)
                np.random.shuffle(depth_inds)
                for d in depth_inds:
                    if flag: break
                    angle, depth, width = offset[v, a, d]
                    if score[v, a, d] > th or score[v, a, d] < 0 or width > max_width:
                        continue
                    R = viewpoint_params_to_matrix(-view, angle)
                    t = target_point
                    gripper = plot_gripper_pro_max(t, R, width, depth, 1.1-score[v, a, d])
                    grippers.append(gripper)
                    flag = True
        if flag:
            cnt += 1
        if cnt == num_grasp:
            break

    vis.add_geometry(model)
    for gripper in grippers:
        vis.add_geometry(gripper)
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.poll_events()
    filename = os.path.join(save_folder, 'object_{}_grasp.png'.format(obj_idx))
    vis.capture_screen_image(filename, do_render=True)
    if show:
        o3d.visualization.draw_geometries([model, *grippers])

def generate_views(N, phi=(np.sqrt(5)-1)/2, center=np.zeros(3, dtype=np.float32), R=1):

    ''' Author: chenxi-wang
    View sampling on a sphere using Febonacci lattices.

    **Input:**

    - N: int, number of viewpoints.

    - phi: float, constant angle to sample views, usually 0.618.

    - center: numpy array of (3,), sphere center.

    - R: float, sphere radius.

    **Output:**

    - numpy array of (N, 3), coordinates of viewpoints.
    '''
    idxs = np.arange(N, dtype=np.float32)
    Z = (2 * idxs + 1) / N - 1
    X = np.sqrt(1 - Z**2) * np.cos(2 * idxs * np.pi * phi)
    Y = np.sqrt(1 - Z**2) * np.sin(2 * idxs * np.pi * phi)
    views = np.stack([X,Y,Z], axis=1)
    views = R * np.array(views) + center
    return views

def get_camera_parameters(camera='kinect'):
    '''
    author: Minghao Gou
    
    **Input:**

    - camera: string of type of camera: 'kinect' or 'realsense'

    **Output:**

    - open3d.camera.PinholeCameraParameters
    '''
    import open3d as o3d
    param = o3d.camera.PinholeCameraParameters()
    param.extrinsic = np.eye(4,dtype=np.float64)
    # param.intrinsic = o3d.camera.PinholeCameraIntrinsic()
    if camera == 'kinect':
        param.intrinsic.set_intrinsics(1280,720,631.5,631.2,639.5,359.5)
    elif camera == 'realsense':
        param.intrinsic.set_intrinsics(1280,720,927.17,927.37,639.5,359.5)
    return param

def get_model_grasps(datapath):
    ''' Author: chenxi-wang
    Load grasp labels from .npz files.
    '''
    label = np.load(datapath)
    points = label['points']
    offsets = label['offsets']
    scores = label['scores']
    collision = label['collision']
    return points, offsets, scores, collision

def viewpoint_params_to_matrix(towards, angle):
    '''
    **Input:**

    - towards: numpy array towards vector with shape (3,).

    - angle: float of in-plane rotation.

    **Output:**

    - numpy array of the rotation matrix with shape (3, 3).
    '''
    axis_x = towards
    axis_y = np.array([-axis_x[1], axis_x[0], 0])
    if np.linalg.norm(axis_y) == 0:
        axis_y = np.array([0, 1, 0])
    axis_x = axis_x / np.linalg.norm(axis_x)
    axis_y = axis_y / np.linalg.norm(axis_y)
    axis_z = np.cross(axis_x, axis_y)
    R1 = np.array([[1, 0, 0],
                   [0, np.cos(angle), -np.sin(angle)],
                   [0, np.sin(angle), np.cos(angle)]])
    R2 = np.c_[axis_x, np.c_[axis_y, axis_z]]
    matrix = R2.dot(R1)
    return matrix.astype(np.float32)

def plot_gripper_pro_max(center, R, width, depth, score=1, color=None):
    '''
    Author: chenxi-wang
    
    **Input:**

    - center: numpy array of (3,), target point as gripper center

    - R: numpy array of (3,3), rotation matrix of gripper

    - width: float, gripper width

    - score: float, grasp quality score

    **Output:**

    - open3d.geometry.TriangleMesh
    '''
    x, y, z = center
    height=0.004
    finger_width = 0.004
    tail_length = 0.04
    depth_base = 0.02
    
    if color is not None:
        color_r, color_g, color_b = color
    else:
        color_r = score # red for high score
        color_g = 0
        color_b = 1 - score # blue for low score
    
    left = create_mesh_box(depth+depth_base+finger_width, finger_width, height)
    right = create_mesh_box(depth+depth_base+finger_width, finger_width, height)
    bottom = create_mesh_box(finger_width, width, height)
    tail = create_mesh_box(tail_length, finger_width, height)

    left_points = np.array(left.vertices)
    left_triangles = np.array(left.triangles)
    left_points[:,0] -= depth_base + finger_width
    left_points[:,1] -= width/2 + finger_width
    left_points[:,2] -= height/2

    right_points = np.array(right.vertices)
    right_triangles = np.array(right.triangles) + 8
    right_points[:,0] -= depth_base + finger_width
    right_points[:,1] += width/2
    right_points[:,2] -= height/2

    bottom_points = np.array(bottom.vertices)
    bottom_triangles = np.array(bottom.triangles) + 16
    bottom_points[:,0] -= finger_width + depth_base
    bottom_points[:,1] -= width/2
    bottom_points[:,2] -= height/2

    tail_points = np.array(tail.vertices)
    tail_triangles = np.array(tail.triangles) + 24
    tail_points[:,0] -= tail_length + finger_width + depth_base
    tail_points[:,1] -= finger_width / 2
    tail_points[:,2] -= height/2

    vertices = np.concatenate([left_points, right_points, bottom_points, tail_points], axis=0)
    vertices = np.dot(R, vertices.T).T + center
    triangles = np.concatenate([left_triangles, right_triangles, bottom_triangles, tail_triangles], axis=0)
    colors = np.array([ [color_r,color_g,color_b] for _ in range(len(vertices))])

    gripper = o3d.geometry.TriangleMesh()
    gripper.vertices = o3d.utility.Vector3dVector(vertices)
    gripper.triangles = o3d.utility.Vector3iVector(triangles)
    gripper.vertex_colors = o3d.utility.Vector3dVector(colors)
    return gripper

def create_mesh_box(width, height, depth, dx=0, dy=0, dz=0):
    ''' Author: chenxi-wang
    Create box instance with mesh representation.
    '''
    box = o3d.geometry.TriangleMesh()
    vertices = np.array([[0,0,0],
                         [width,0,0],
                         [0,0,depth],
                         [width,0,depth],
                         [0,height,0],
                         [width,height,0],
                         [0,height,depth],
                         [width,height,depth]])
    vertices[:,0] += dx
    vertices[:,1] += dy
    vertices[:,2] += dz
    triangles = np.array([[4,7,5],[4,6,7],[0,2,4],[2,6,4],
                          [0,1,2],[1,3,2],[1,5,7],[1,7,3],
                          [2,3,7],[2,7,6],[0,4,1],[1,4,5]])
    box.vertices = o3d.utility.Vector3dVector(vertices)
    box.triangles = o3d.utility.Vector3iVector(triangles)
    return box

if __name__=="__main__":

    root = "./data"
    save_folder = "./result"
    vis(root, obj_idx=1, show=True, num_grasp=50)
