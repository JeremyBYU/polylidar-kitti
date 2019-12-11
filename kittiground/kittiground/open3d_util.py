import numpy as np
import open3d as o3d
import time

from kittiground import EXTRINSICS


MAX_POLYS = 10
ORANGE = (255/255, 188/255, 0)
GREEN = (0, 255, 0)

def update_points(pcd, pc):
    pcd.points = o3d.utility.Vector3dVector(pc)

def set_line(line_set, points, lines, colors):
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

def update_polygon(line_set, linear_ring, color=[1, 0, 0], origin=None):
    points = np.array(linear_ring)
    if origin is not None:
        points = points - origin
    n_points = points.shape[0]

    lines = [[i, i + 1] for i in range(n_points -1)]
    colors = [list(color)] * (n_points - 1)
    set_line(line_set, points, lines, colors)

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


def handle_polygon(polygon_dict, all_polys, poly_counter, origin=None):
    polygon = polygon_dict['data']
    color = polygon_dict.get('color', GREEN)
    if poly_counter >= MAX_POLYS:
        return poly_counter

    polygon_gl = all_polys[poly_counter]
    update_polygon(polygon_gl, polygon.exterior, color=color, origin=origin)
    poly_counter += 1
    # logging.info("Number of Holes: %s", len(polygon.interiors))
    for hole in polygon.interiors:
        if poly_counter >= MAX_POLYS:
            return poly_counter
        polygon_gl = all_polys[poly_counter]
        color = ORANGE
        update_polygon(polygon_gl, hole, color=color, origin=origin)
        poly_counter += 1
    return poly_counter

def handle_shapes(planes, obstacles, all_polys):
    poly_counter = 0
    for plane, plane_height in planes:
        plane_dict = dict(data=plane)
        poly_counter = handle_polygon(plane_dict, all_polys, poly_counter, origin=None)
    for plane, plane_height in obstacles:
        plane_dict = dict(data=plane, color=ORANGE)
        poly_counter = handle_polygon(plane_dict, all_polys, poly_counter, origin=None)

    for i in range(poly_counter, MAX_POLYS):
        polygon_gl = all_polys[i]
        polygon_gl.clear()


def set_initial_view(vis, extrinsics=EXTRINSICS):
    ctr = vis.get_view_control()
    camera_params = ctr.convert_to_pinhole_camera_parameters()
    # print(camera_params.intrinsic.intrinsic_matrix)
    # print(camera_params.extrinsic)
    camera_params.extrinsic = EXTRINSICS
    ctr.convert_from_pinhole_camera_parameters(camera_params)

def init_vis(fov_step=-40, width=1242, height=int(375*2)):
    vis = o3d.visualization.Visualizer()
    vis.create_window("3D Viewer", width, height)
    
    # print("Field of view (before changing) %.2f" % ctr.get_field_of_view())
    # ctr.change_field_of_view(step=fov_step)
    # print("Field of view (after changing) %.2f" % ctr.get_field_of_view())

    # create point cloud
    pcd = o3d.geometry.PointCloud()
    # create empty polygons
    all_polys = []
    for i in range(MAX_POLYS):
        polygon1_gl = o3d.geometry.LineSet()
        all_polys.append(polygon1_gl)
    # create axis frame
    axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    # Create grid
    grid_ls = o3d.geometry.LineSet()
    my_grid = grid(size=20, n=20, plane='xy', plane_offset=-1.63, translate=[0, 10, 0])
    set_line(grid_ls, *my_grid)

    vis.add_geometry(pcd)
    vis.add_geometry(axis_frame)
    vis.add_geometry(grid_ls)
    # add polygons, initially empty polygons
    for poly in all_polys:
        vis.add_geometry(poly)

    return vis, pcd, all_polys