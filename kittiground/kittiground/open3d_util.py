import numpy as np
import open3d as o3d
import time

from kittiground import EXTRINSICS
from kittiground.kittiground.line_mesh import LineMesh


MAX_POLYS = 10
ORANGE = (255/255, 188/255, 0)
GREEN = (0, 255/255, 0)

def update_points(pcd, pc):
    pcd.points = o3d.utility.Vector3dVector(pc)

def set_line(line_set, points, lines, colors):
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

# def create_grid()
def grid(size=10, n=10, color=[0.5, 0.5, 0.5], plane='xy', plane_offset=-1, translate=[0, 0, 0]):
    """draw a grid on xz plane"""

    # lineset = o3d.geometry.LineSet()
    s = size / float(n)
    s2 = 0.5 * size
    points = []

    for i in range(0, n + 1):
        x = -s2 + i * s
        points.append([x, -s2, plane_offset])
        points.append([x, s2, plane_offset])
    for i in range(0, n + 1):
        z = -s2 + i * s
        points.append([-s2, z, plane_offset])
        points.append([s2, z, plane_offset])

    points = np.array(points)
    if plane == 'xz':
        points[:,[2,1]] = points[:,[1,2]]

    points = points + translate

    n_points = points.shape[0]
    lines = [[i, i + 1] for i in range(0, n_points -1, 2)]
    colors = [list(color)] * (n_points - 1)
    return points, lines, colors

def clear_polys(all_polys, vis):
    for line_mesh in all_polys:
        line_mesh.remove_line(vis)
    return []

def handle_shapes(vis, planes, obstacles, all_polys, line_radius=0.15):
    all_polys = clear_polys(all_polys, vis)
    for plane, _ in planes:
        points = np.array(plane.exterior)
        line_mesh = LineMesh(points, colors=GREEN, radius=line_radius)
        line_mesh.add_line(vis)
        all_polys.append(line_mesh)

    for plane, _ in obstacles:
        points = np.array(plane.exterior)
        line_mesh = LineMesh(points, colors=ORANGE, radius=line_radius)
        line_mesh.add_line(vis)
        all_polys.append(line_mesh)

    return all_polys

def get_extrinsics(vis):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    return camera_params.extrinsic

def set_initial_view(vis, extrinsics=EXTRINSICS):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    # print(camera_params.intrinsic.intrinsic_matrix)
    # print(camera_params.extrinsic)
    camera_params.extrinsic = extrinsics
    ctr.convert_from_pinhole_camera_parameters(camera_params)

def init_vis(fov_step=-40, width=1242, height=int(375*2)):
    vis = o3d.visualization.Visualizer()
    vis.create_window("3D Viewer", width, height)

    # create point cloud
    pcd = o3d.geometry.PointCloud()
    # create empty polygons list
    all_polys = []
    # Create axis frame
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    # Create grid
    grid_ls = o3d.geometry.LineSet()
    my_grid = grid(size=20, n=20, plane='xy', plane_offset=-1.63, translate=[0, 10, 0])
    set_line(grid_ls, *my_grid)

    vis.add_geometry(pcd)
    vis.add_geometry(axis_frame)
    vis.add_geometry(grid_ls)

    return vis, pcd, all_polys