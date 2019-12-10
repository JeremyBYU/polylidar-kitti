import time
import logging

import pykitti
from pykitti.utils import rotz, OxtsData, transform_from_rot_trans
import cv2
import numpy as np
import open3d as o3d

from polylidar import extractPolygons
from kittiground.kittiground.open3d_util import init_vis
from kittiground.grounddetector import align_vector_to_zaxis, filter_planes_and_holes

np.set_printoptions(suppress=True,
   formatter={'float_kind':'{:.8f}'.format})

class KittiGround(object):
    def __init__(self, config):
        super().__init__()
        self.data_folder: str = config['data_folder']
        self.date: str = config['date']
        self.drive: str = config['drive']
        self.frames = config['frames']
        self.view_image = config['view_image']
        self.view_3D = config['view_3D']
        self.polylidar_kwargs = config['polygon']['polylidar']
        self.postprocess = config['polygon']['postprocess']

        self.load_kitti(self.data_folder, self.date,
                        self.drive, frames=self.frames)

        self.frame_iter = range(len(self.data_kitti.velo_files))

    def load_kitti(self, data_folder: str, date: str, drive: str, **kwargs):
        self.data_kitti = pykitti.raw(data_folder, date, drive, **kwargs)
        # print(self.data_kitti.calib)
        # print(dir(self.data_kitti.calib))
        self.load_projections(self.view_image['cam'])
        self.fix_imu()
        self.img_n = 1242
        self.img_m = 375

    def load_projections(self, camN: str):
        self.R00 = self.data_kitti.calib.R_rect_00
        if camN == "cam0":
            T_cam_velo = self.data_kitti.calib.T_cam0_velo
            T_cam_imu = self.data_kitti.calib.T_cam0_imu
            P_rect = self.data_kitti.calib.P_rect_00
            get_cam_fn = getattr(self.data_kitti, 'get_cam0')
        elif camN == "cam1":
            T_cam_velo = self.data_kitti.calib.T_cam1_velo
            T_cam_imu = self.data_kitti.calib.T_cam2_imu
            P_rect = self.data_kitti.calib.P_rect_10
            get_cam_fn = getattr(self.data_kitti, 'get_cam1')
        elif camN == "cam2":
            T_cam_velo = self.data_kitti.calib.T_cam2_velo
            T_cam_imu = self.data_kitti.calib.T_cam2_imu
            P_rect = self.data_kitti.calib.P_rect_20
            get_cam_fn = getattr(self.data_kitti, 'get_cam2')
        elif camN == "cam3":
            T_cam_velo = self.data_kitti.calib.T_cam3_velo
            T_cam_imu = self.data_kitti.calib.T_cam3_imu
            P_rect = self.data_kitti.calib.P_rect_30
            get_cam_fn = getattr(self.data_kitti, 'get_cam3')

        self.T_cam_velo = T_cam_velo
        self.P_rect = P_rect
        self.get_cam_fn = get_cam_fn
        self.T_cam_imu = T_cam_imu
        self.T_velo_imu = self.data_kitti.calib.T_velo_imu

    def fix_imu(self):
        new_oxts = []
        for oxts in self.data_kitti.oxts:
            T = oxts.T_w_imu
            reverse_yaw  = rotz(oxts.packet.yaw).T
            reverse_yaw = transform_from_rot_trans(reverse_yaw, np.array([0, 0, 0]))
            T = reverse_yaw @ T
            new_oxts.append(OxtsData(oxts.packet, T))
        self.data_kitti.oxts = new_oxts

    def get_velo(self, frame_idx):
        pts3D_velo_unrectified = self.data_kitti.get_velo(frame_idx)
        # create copy of intensity data
        intensity = np.copy(pts3D_velo_unrectified[:, 3])
        # use intensity data to hold 1 hordiante for homgoneous transfrom
        pts3D_velo_unrectified[:, 3] = 1
        pts3D_velo_unrectified = pts3D_velo_unrectified.transpose()
        pts3D_cam_rect = self.T_cam_velo @pts3D_velo_unrectified
        # only points in front of the camera
        idx = (pts3D_cam_rect[2, :] > 0)
        pts3D_cam_rect_filt = np.ascontiguousarray(pts3D_cam_rect[:, idx])
        intensity_filt = np.ascontiguousarray(intensity[idx])
        return pts3D_cam_rect_filt, intensity_filt

    def project_points(self, pts3D_cam_rect):
        pts2D_cam_rect = self.P_rect @ pts3D_cam_rect
        pts2D_cam_rect[0, :] = pts2D_cam_rect[0, :] / pts2D_cam_rect[2, :]
        pts2D_cam_rect[1, :] = pts2D_cam_rect[1, :] / pts2D_cam_rect[2, :]

        idx = (pts2D_cam_rect[0, :] >= 0) & (pts2D_cam_rect[0, :] < self.img_n) & \
            (pts2D_cam_rect[1, :] >= 0) & (pts2D_cam_rect[1, :] < self.img_m)

        pts2D_cam_rect_filt = np.ascontiguousarray(
            pts2D_cam_rect[:, idx].astype(np.int))

        return pts2D_cam_rect_filt, idx
        # intensity_filt = intensity[idx]
        #  = pts3D_velo_unrectified[]

    @staticmethod
    def plot_points(image, points, color):
        """ plot project velodyne points into camera image """
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        radius = 2
        for i in range(points.shape[1]):
            pt_2d = (points[0, i], points[1, i])
            c = (int(color[i]), 255, 255)
            cv2.circle(hsv_image, pt_2d, radius, c, -1)

        return cv2.cvtColor(hsv_image, cv2.COLOR_HSV2RGB)

    def get_polygon(self, points3D_cam, color):
        """ plot project velodyne points into camera image """
        points3D_rot, rm = align_vector_to_zaxis(points3D_cam, np.array([0, 1, 0]))
        points3D_rot_ = np.ascontiguousarray(points3D_rot[:, :3])
        logging.info("Extracting Polygons from point cloud of size: %d", points3D_rot.shape[0])
        t1 = time.time()
        polygons = extractPolygons(points3D_rot_, **self.polylidar_kwargs)
        t2 = time.time()
        planes, obstacles = filter_planes_and_holes(polygons, points3D_rot_, self.postprocess)
        t3 = time.time()
        return points3D_rot

    @staticmethod
    def normalize_data(data, scale=255):
        """ Normalize data """
        data_min = np.min(data)
        data_max = np.max(data)
        return ((data - data_min) / (data_max - data_min) * scale).astype(np.uint8)

    def load_frame(self, frame_idx: int):
        """Load frame from kitti

        Arguments:
            frame_idx {int} -- Frame index

        Returns:
            (img, pts2D, pts2D_color, pts3D) -- M X N ndarray image, projected lidar points, color for points, velodyne points
        """
        imgN = np.asarray(self.get_cam_fn(frame_idx)) # image
        pts3D_cam, intensity = self.get_velo(frame_idx) # 3D points
        oxts = self.data_kitti.oxts[frame_idx] # imu

        # project points
        pts2D_cam, idx = self.project_points(pts3D_cam)

        # filter points outside of image
        pts3D_cam = pts3D_cam[:, idx].T[:, :3]
        intensity = intensity[idx]

        pose_cam = self.T_velo_imu @ oxts.T_w_imu
        print(pose_cam)
        print("Roll: {:.3f}; Pitch: {:.3f}; Yaw: {:.3f}".format(np.degrees(oxts.packet.roll), np.degrees(oxts.packet.pitch), np.degrees(oxts.packet.yaw)))

        if self.view_image['pointcloud']['color'] == 'intensity':
            color = intensity
        elif self.view_image['pointcloud']['color'] == 'distance':
            distance = np.sqrt(
                pts3D_cam[:, 0] ** 2 + pts3D_cam[:, 1] ** 2 + pts3D_cam[:, 2] ** 2)
            color = distance

        color = self.normalize_data(color)

        return imgN, pts2D_cam, color, pts3D_cam

    def run(self):
        vis, pcd = init_vis()
        for frame_idx in self.frame_iter:
            img, pts2D_cam, color, pts3D_cam = self.load_frame(frame_idx)
            pts3D_std = self.get_polygon(pts3D_cam, color)
            img_points = self.plot_points(img, pts2D_cam, color)
            cv2.imshow(
                'Image View - {}'.format(self.view_image['cam']), img_points)
            cv2.waitKey(1)
            pcd.points = o3d.utility.Vector3dVector(pts3D_std)
            vis.update_geometry()

            while(True):
                vis.poll_events()
                vis.update_renderer()
                res = cv2.waitKey(33)
                if res != -1:
                    break
                # print(res)

        # img = self.data_kitti.
