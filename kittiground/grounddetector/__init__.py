import sys
import math
import time
import logging

import numpy as np
import cv2

from shapely.geometry import Polygon

M2TOCM2 = 10000
CMTOM = 0.01

ORANGE = [249, 115, 6]


def create_3D_coords(poly, height=0):
    pts = np.array(poly.exterior.coords)  # NX2
    if pts.shape[1] > 2:
        return poly
    pts = np.column_stack((pts, np.ones((pts.shape[0])) * height))  # NX3
    shell = pts.tolist()
    holes = []
    for hole in poly.interiors:
        pts = np.array(hole.coords)
        pts = np.column_stack((pts, np.ones((pts.shape[0])) * height))  # NX3
        holes.append(pts.tolist())
    # print(shell, holes)
    return Polygon(shell=shell, holes=holes)


def rotation_matrix(x_theta=90):
    theta_rad = math.radians(x_theta)
    rotation_matrix = np.array([[1, 0, 0], [0, math.cos(theta_rad), -math.sin(theta_rad)],
                                [0, math.sin(theta_rad), math.cos(theta_rad)]])
    return rotation_matrix


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


def filter_zero(points_np):
    """
    Filter out all zero vectors (3D)
    """
    # TODO replace with cython or numba
    A = points_np[:, 0]
    B = points_np[:, 1]
    C = points_np[:, 2]
    t0 = time.time()
    mask = A == 0.0
    mask = mask & (B == 0.0)
    mask = mask & (C == 0.0)
    points_np = points_np[~mask]
    # print(f"Filtering Took {(time.time() - t0) * 1000:.1f} ms")
    return points_np


def rotate_points(points, rot):
    """
    Rotate 3D points given a provided rotation matrix
    """
    points_rot = points.transpose()
    points_rot = rot @ points_rot
    points_rot = points_rot.transpose()
    # print(f"Rotation Took {(time.time() - t0) * 1000:.1f} ms")
    return points_rot


def get_normal(points):
    points = points - np.mean(points, axis=0)
    u, s, vh = np.linalg.svd(points, compute_uv=True)
    # GET THE LAST ROW!!!!!!!NOT LAST COLUMN
    return vh[-1, :]


def calculate_plane_normal(patches):
    """
    Get normal of all the patches
    """
    normals = []
    for patch in patches:
        normal = get_normal(patch)
        normals.append(normal)
    # Taken naive mean of normals
    # TODO outlier removal
    normals = np.mean(np.array(normals), axis=0)
    return normals


def align_vector_to_zaxis(points, vector=np.array([0, 0, 1])):
    """
    Aligns z axis frame to chosen vector
    """
    # Shortcut from computing cross product of -Z axis X vector
    axis = np.cross(vector, np.array([0, 0, -1]))
    axis = axis / np.linalg.norm(axis)
    angle = math.acos(-vector[2])

    rm = axis_angle_rm(axis, angle)
    points_rot = rotate_points(points, rm)
    return points_rot, rm


def get_point(pi, points):
    return [points[pi, 0], points[pi, 1], points[pi, 2]]


def filter_planes_and_holes(polygons, points, config_pp):
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
        shell_coords = [get_point(pi, points) for pi in poly.shell]
        outline = Polygon(shell=shell_coords)

        outline = outline.buffer(distance=config_pp['buffer'])
        outline = outline.simplify(tolerance=config_pp['simplify'])
        area = outline.area
        if area >= post_filter['plane_area']['min']:
            # Capture the polygon as well as its z height
            z_value = shell_coords[0][2]
            planes.append((create_3D_coords(outline, z_value), z_value))

            for hole_poly in poly.holes:
                # Filter by number of obstacle vertices, removes noisy holes
                if len(hole_poly) > post_filter['hole_vertices']['min']:
                    shell_coords = [get_point(pi, points) for pi in hole_poly]
                    outline = Polygon(shell=shell_coords)
                    area = outline.area
                    # filter by area
                    if area >= post_filter['hole_area']['min'] and area < post_filter['hole_area']['max']:
                        outline = outline.buffer(distance=config_pp['buffer'])
                        outline = outline.simplify(
                            tolerance=config_pp['simplify'])
                        z_value = shell_coords[0][2]
                        obstacles.append(
                            (create_3D_coords(outline, z_value), z_value))
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
        height, color=ORANGE,  thickness=thickness)
    return color_image
