import sys
import math
import time
import logging

import numpy as np
from scipy import spatial
import cv2
from shapely.geometry import Polygon, JOIN_STYLE

from polylidar import extractPolygons

M2TOCM2 = 10000
CMTOM = 0.01

ORANGE = [249, 115, 6]
ORANGE_BGR = [6, 115, 249]




def axis_angle_rm(axis=np.array([1, 0, 0]), angle=-1.57):
    """
    Create rotation matrix given an axis and angle
    https://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
    """
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    x, y, z = axis[0], axis[1], axis[2]
    rotation_matrix = np.array(
        [
            [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
            [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
            [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
        ])
    return rotation_matrix


# def filter_zero(points_np):
#     """
#     Filter out all zero vectors (3D)
#     """
#     # TODO replace with cython or numba
#     A = points_np[:, 0]
#     B = points_np[:, 1]
#     C = points_np[:, 2]
#     t0 = time.time()
#     mask = A == 0.0
#     mask = mask & (B == 0.0)
#     mask = mask & (C == 0.0)
#     points_np = points_np[~mask]
#     # print(f"Filtering Took {(time.time() - t0) * 1000:.1f} ms")
#     return points_np


def rotate_points(points, rot):
    """
    Rotate 3D points given a provided rotation matrix
    """
    points_rot = points.transpose()
    points_rot = rot @ points_rot
    points_rot = points_rot.transpose()
    # print(f"Rotation Took {(time.time() - t0) * 1000:.1f} ms")
    return points_rot


# def get_normal(points):
#     points = points - np.mean(points, axis=0)
#     u, s, vh = np.linalg.svd(points, compute_uv=True)
#     # GET THE LAST ROW!!!!!!!NOT LAST COLUMN
#     return vh[-1, :]


# def calculate_plane_normal(patches):
#     """
#     Get normal of all the patches
#     """
#     normals = []
#     for patch in patches:
#         normal = get_normal(patch)
#         normals.append(normal)
#     # Taken naive mean of normals
#     # TODO outlier removal
#     normals = np.mean(np.array(normals), axis=0)
#     return normals


def plot_points(image, points, color):
    """ plot projected velodyne points into camera image """
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    radius = 4
    for i in range(points.shape[1]):
        pt_2d = (points[0, i], points[1, i])
        c = (int(color[i]), 255, 255)
        cv2.circle(hsv_image, pt_2d, radius, c, -1)

    return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)

def align_vector_to_axis(points, vector=np.array([0, 0, 1]), axis=[0, 0, -1], ):
    """
    Aligns z axis frame to chosen vector
    """
    # Shortcut from computing cross product of -Z axis X vector
    axis_ = np.cross(vector, np.array(axis))
    axis_ = axis_ / np.linalg.norm(axis_)
    angle = math.acos(-vector[2])

    rm = axis_angle_rm(axis_, angle)
    points_rot = rotate_points(points, rm)
    return points_rot, rm


# def get_point(pi, points):
#     return [points[pi, 0], points[pi, 1], points[pi, 2]]

def get_points(point_idxs, points):
    return points[point_idxs, :]

def create_kd_tree(shell_coords, hole_coords):
    hole_coords.append(shell_coords)
    all_vertices = np.vstack(hole_coords)
    kd_tree = spatial.KDTree(all_vertices, leafsize=100)
    return kd_tree

def add_column(array, z_value):
    ones = np.ones((array.shape[0], 1)) * z_value
    stacked = np.column_stack((array, ones))
    return stacked

def recover_3d(poly, kd_tree, z_value):
    shell_3D = add_column(np.array(poly.exterior), z_value)
    # print(shell_3D.shape)
    d, shell_idx = kd_tree.query(shell_3D)
    # print(shell_idx.shape)
    kd_data = kd_tree.data[shell_idx,:]
    # print(kd_data.shape)
    shell_3D[:, 2] = kd_data[:, 2]
    holes_lr = []
    for hole in poly.interiors:
        hole_lr = add_column(np.array(hole), z_value)
        d, shell_idx = kd_tree.query(hole_lr)
        kd_data = kd_tree.data[shell_idx,:]
        hole_lr[:, 2] = kd_data[:, 2]
        holes_lr.append(hole_lr)
    
    poly_3d = Polygon(shell=shell_3D, holes=holes_lr)
    return poly_3d
    # print(poly.exterior)
    # print(poly_3d.exterior)


def get_polygon(points3D_cam, polylidar_kwargs, postprocess_config):
    """Gets polygons from pont cloud

    Arguments:
        points3D_cam {ndarray} -- Point cloud in Camera Frame

    Returns:
        (ndarray, ndarray, list[Polygons], list[Polygons], tuple(ints)) -- 
        Rotated point cloud, 
        rotation matrix from camera frame to rotated frame
        list of shapley polygons for planes and obstacles
        a tuple of execution times

    """
    t0 = time.time()
    points3D_rot, rm = align_vector_to_axis(
        points3D_cam, np.array([0, 1, 0]))
    points3D_rot_ = np.ascontiguousarray(points3D_rot[:, :3])
    logging.debug(
        "Extracting Polygons from point cloud of size: %d", points3D_rot.shape[0])
    t1 = time.time()
    polygons = extractPolygons(points3D_rot_, **polylidar_kwargs)
    t2 = time.time()
    planes, obstacles = filter_planes_and_holes2(
        polygons, points3D_rot_, postprocess_config)
    logging.debug("Number of Planes: %d; Number of obstacles: %d",
                    len(planes), len(obstacles))
    t3 = time.time()

    t_rotation = (t1 - t0) * 1000
    t_polylidar = (t2 - t1) * 1000
    t_polyfilter = (t3 - t2) * 1000
    times = (t_rotation, t_polylidar, t_polyfilter)
    return points3D_rot, rm, planes, obstacles, times

def filter_planes_and_holes2(polygons, points, config_pp):
    """Extracts the plane and obstacles returned from polylidar
    Will filter polygons according to: number of vertices and size
    Will also buffer (dilate) and simplify polygons

    Arguments:
        polygons {list[Polygons]} -- A list of polygons returned from polylidar
        points {ndarray} -- MX3 array
        config_pp {dict} -- Configuration for post processing filtering

    Returns:
        tuple -- A list of plane shapely polygons and a list of obstacle polygons
    """
    # filtering configuration
    post_filter = config_pp['filter']

    # will hold the plane(s) and obstacles found
    planes = []
    obstacles = []
    for poly in polygons:
        # shell_coords = [get_point(pi, points) for pi in poly.shell]
        shell_coords = get_points(poly.shell, points)
        hole_coords = [get_points(hole, points) for hole in poly.holes]
        poly_shape = Polygon(shell=shell_coords, holes=hole_coords)
        area = poly_shape.area
        if area < post_filter['plane_area']['min']:
            continue
        z_value = shell_coords[0][2]
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(tolerance=config_pp['simplify'], preserve_topology=False)
        # Perform 2D geometric operations
        if config_pp['buffer'] or config_pp['positive_buffer']:
            # poly_shape = poly_shape.buffer(-config_pp['buffer'], 1, join_style=JOIN_STYLE.mitre).buffer(config_pp['buffer'], 1, join_style=JOIN_STYLE.mitre)
            poly_shape = poly_shape.buffer(config_pp['positive_buffer'], join_style=JOIN_STYLE.mitre, resolution=4)
            poly_shape = poly_shape.buffer(distance=-config_pp['buffer'] * 3, resolution=4)
            poly_shape = poly_shape.buffer(distance=config_pp['buffer'] * 2, resolution=4)
        if config_pp['simplify']:
            poly_shape = poly_shape.simplify(tolerance=config_pp['simplify'], preserve_topology=False)
        
        # Its possible that our polygon has no broken into a multipolygon
        # Check for this situation and handle it
        all_poly_shapes = [poly_shape]
        if poly_shape.geom_type == 'MultiPolygon':
            all_poly_shapes = list(poly_shape.geoms)
            all_poly_shapes = sorted(all_poly_shapes, key=lambda geom: geom.area, reverse=True)
            all_poly_shapes = all_poly_shapes[:1]

        # iteratre through every polygons and check for plane extraction
        for poly_shape in all_poly_shapes:
            area = poly_shape.area
            if area >= post_filter['plane_area']['min']:
                if config_pp['buffer'] or config_pp['simplify'] or config_pp['positive_buffer']:
                    # convert back to 3D coordinates
                    # create kd tree for vertex lookup after buffering operations
                    kd_tree = create_kd_tree(shell_coords, hole_coords)
                    poly_shape = recover_3d(poly_shape, kd_tree, z_value)
                pass
                # Capture the polygon as well as its z height
                new_plane_polygon = Polygon(shell=poly_shape.exterior)
                planes.append((new_plane_polygon, z_value))

                for hole_lr in poly_shape.interiors:
                    # Filter by number of obstacle vertices, removes noisy holes
                    if len(hole_lr.coords) > post_filter['hole_vertices']['min']:
                        hole_poly = Polygon(shell=hole_lr)
                        area = hole_poly.area
                        # filter by area
                        if area >= post_filter['hole_area']['min'] and area < post_filter['hole_area']['max']:
                            z_value = hole_lr.coords[0][2]
                            obstacles.append((hole_poly, z_value))
    return planes, obstacles

def project_points(pts3D_cam_rect, proj_matrix, img_m, img_n):
    pts2D_cam_rect = proj_matrix @ pts3D_cam_rect

    # Remove pixels that are outside the image
    pts2D_cam_rect[0, :] = pts2D_cam_rect[0, :] / pts2D_cam_rect[2, :]
    pts2D_cam_rect[1, :] = pts2D_cam_rect[1, :] / pts2D_cam_rect[2, :]

    idx = (pts2D_cam_rect[0, :] >= 0) & (pts2D_cam_rect[0, :] < img_n) & \
        (pts2D_cam_rect[1, :] >= 0) & (pts2D_cam_rect[1, :] < img_m)

    pts2D_cam_rect_filt = np.ascontiguousarray(
        pts2D_cam_rect[:, idx].astype(np.int))

    return pts2D_cam_rect_filt, idx


def get_pix_coordinates(pts, proj_mat, w, h):
    """Get Pixel coordinates of ndarray

    Arguments:
        pts {ndarray} -- 3D point clouds 3XN
        proj_mat {ndarray} -- 4X3 Projection Matrix
        w {int} -- width
        h {int} -- height

    Returns:
        ndarray -- Pixel coordinates
    """
    points_t = np.ones(shape=(4, pts.shape[1]))
    points_t[:3, :] = pts
    pixels, idx = project_points(points_t, proj_mat, h, w)
    pixels = np.ascontiguousarray(pixels[:2, :])
    logging.debug("Pixels Shape %r", pixels.shape)
    return pixels


def plot_opencv_polys(polygons, color_image, proj_mat, rot_mat, w, h, color=(0, 255, 0), thickness=2):
    for i, (poly, height) in enumerate(polygons):
        # Get 2D polygons and assign z component the height value of extracted plane
        pts = np.array(poly.exterior.coords)  # NX2
        # pts = np.column_stack((pts, np.ones((pts.shape[0])) * height))  # NX3
        # Transform polylidar plane coordinate system (z-up) to original cordinate system of camera frame
        pts = pts.transpose()  # 3XN
        pts = np.linalg.inv(rot_mat) @ pts

        # Project coordinates to image space
        pix_coords = get_pix_coordinates(pts, proj_mat, w, h).T
        pix_coords = pix_coords.reshape((-1, 1, 2))
        cv2.polylines(color_image, [pix_coords],
                      True, color, thickness=thickness)
    return color_image


def plot_planes_and_obstacles(planes, obstacles, proj_mat, rot_mat, color_image, width, height, thickness=2):
    """Plots the planes and obstacles (3D polygons) into the color image

    Arguments:
        planes {list(Polygons)} -- List of Shapely Polygon with height tuples
        obstacles {list[(polygon, height)]} -- List of tuples with polygon with height
        proj_mat {ndarray} -- Projection Matrix
        rot_mat {ndarray} -- Rotation Matrix
        color_image {ndarray} -- Color Image
        width {int} -- width of image
        height {int} -- height of image
    """
    color_image = plot_opencv_polys(
        planes, color_image, proj_mat, rot_mat, width,
        height, color=(0, 255, 0), thickness=thickness)

    color_image = plot_opencv_polys(
        obstacles, color_image, proj_mat, rot_mat, width,
        height, color=ORANGE_BGR,  thickness=thickness)
    return color_image
