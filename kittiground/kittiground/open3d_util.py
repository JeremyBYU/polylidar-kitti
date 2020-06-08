import time
from copy import deepcopy

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import matplotlib.colors as colors

from kittiground import EXTRINSICS
from kittiground.kittiground.line_mesh import LineMesh

COLOR_PALETTE = list(map(colors.to_rgb, plt.rcParams['axes.prop_cycle'].by_key()['color']))
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

def create_open_3d_mesh(triangles, points, triangle_normals=None, color=COLOR_PALETTE[0], counter_clock_wise=True):
    """Create an Open3D Mesh given triangles vertices

    Arguments:
        triangles {ndarray} -- Triangles array
        points {ndarray} -- Points array

    Keyword Arguments:
        color {list} -- RGB COlor (default: {[1, 0, 0]})

    Returns:
        mesh -- Open3D Mesh
    """
    mesh_2d = o3d.geometry.TriangleMesh()
    if points.ndim == 1:
        points = points.reshape((int(points.shape[0] / 3), 3))
    if triangles.ndim == 1:
        triangles = triangles.reshape((int(triangles.shape[0] / 3), 3))
        # Open 3D expects triangles to be counter clockwise
    if not counter_clock_wise:
        triangles = np.ascontiguousarray(np.flip(triangles, 1))
    mesh_2d.triangles = o3d.utility.Vector3iVector(triangles)
    mesh_2d.vertices = o3d.utility.Vector3dVector(points)
    if triangle_normals is None:
        mesh_2d.compute_vertex_normals()
        mesh_2d.compute_triangle_normals()
    elif triangle_normals.ndim == 1:
        triangle_normals_ = triangle_normals.reshape((int(triangle_normals.shape[0] / 3), 3))
        mesh_2d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals_)
    else:
        mesh_2d.triangle_normals = o3d.utility.Vector3dVector(triangle_normals)
    mesh_2d.paint_uniform_color(color)
    mesh_2d.compute_vertex_normals()
    return mesh_2d

def get_colors(inp, colormap=plt.cm.viridis, vmin=None, vmax=None):
    norm = plt.Normalize(vmin, vmax)
    return colormap(norm(inp))

def split_triangles(mesh):
    """
    Split the mesh in independent triangles    
    """
    triangles = np.asarray(mesh.triangles).copy()
    vertices = np.asarray(mesh.vertices).copy()

    triangles_3 = np.zeros_like(triangles)
    vertices_3 = np.zeros((len(triangles) * 3, 3), dtype=vertices.dtype)

    for index_triangle, t in enumerate(triangles):
        index_vertex = index_triangle * 3
        vertices_3[index_vertex] = vertices[t[0]]
        vertices_3[index_vertex + 1] = vertices[t[1]]
        vertices_3[index_vertex + 2] = vertices[t[2]]

        triangles_3[index_triangle] = np.arange(index_vertex, index_vertex + 3)

    mesh_return = deepcopy(mesh)
    mesh_return.triangles = o3d.utility.Vector3iVector(triangles_3)
    mesh_return.vertices = o3d.utility.Vector3dVector(vertices_3)
    mesh_return.triangle_normals = mesh.triangle_normals
    mesh_return.paint_uniform_color([0.5, 0.5, 0.5])
    return mesh_return

def assign_vertex_colors(mesh, normal_colors, mask=None):
    """Assigns vertex colors by given normal colors
    NOTE: New mesh is returned

    Arguments:
        mesh {o3d:TriangleMesh} -- Mesh
        normal_colors {ndarray} -- Normals Colors

    Returns:
        o3d:TriangleMesh -- New Mesh with painted colors
    """
    split_mesh = split_triangles(mesh)
    vertex_colors = np.asarray(split_mesh.vertex_colors)
    triangles = np.asarray(split_mesh.triangles)
    if mask is not None:
        triangles = triangles[mask, :]
    for i in range(triangles.shape[0]):
        # import ipdb; ipdb.set_trace()
        color = normal_colors[i, :]
        p_idx = triangles[i, :]
        vertex_colors[p_idx] = color

    split_mesh.compute_vertex_normals()
    return split_mesh