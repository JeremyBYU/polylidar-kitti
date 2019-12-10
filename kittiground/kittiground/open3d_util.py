import numpy as np
import open3d as o3d
import time


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


def init_vis():
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    # create point cloud
    pcd = o3d.geometry.PointCloud()
    # create axis frame
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    # Create grid
    grid_ls = o3d.geometry.LineSet()
    my_grid = grid(size=20, n=30, plane='xy', plane_offset=-1.63, translate=[0, 10, 0])
    set_line(grid_ls, *my_grid)

    vis.add_geometry(pcd)
    vis.add_geometry(axis_frame)
    vis.add_geometry(grid_ls)

    return vis, pcd