import open3d as o3d

pc_file = "./result/pc2.pcd"
pc = o3d.io.read_point_cloud(pc_file)
o3d.visualization.draw_geometries([pc])