import time
import logging
from pathlib import Path

import pykitti
from pykitti.utils import rotz, OxtsData, transform_from_rot_trans
import cv2
import numpy as np
import open3d as o3d

from kittiground import EXTRINSICS, IMG_WIDTH, IMG_HEIGHT
from kittiground.kittiground.open3d_util import init_vis, handle_shapes, set_initial_view
from kittiground.grounddetector import filter_planes_and_holes2, plot_planes_and_obstacles, project_points, get_polygon, plot_points

np.set_printoptions(suppress=True,
                    formatter={'float_kind': '{:.8f}'.format})


def moving_average(a, n=3, pad_start=None):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    if pad_start is not None:
        ret = ret / n
        for i in range(n-1):
            ret[i] = pad_start
        return ret
    else:
        return ret[n - 1:] / n


class KittiGround(object):
    def __init__(self, config):
        super().__init__()
        self.data_folder: str = config['data_folder']
        self.date: str = config['date']
        self.drive: str = config['drive']
        self.frames = config['frames']
        self.view_image = config['view_image']
        self.cam = config['cam']
        self.pointcloud = config['pointcloud']
        self.view_3D = config['view_3D']
        self.polylidar_kwargs = config['polygon']['polylidar']
        self.postprocess = config['polygon']['postprocess']
        self.record = config['record']
        self.record['fpath'] = str(Path(self.record['directory']) / "{}_{}.mp4".format(self.date, self.drive))
        self.interactive = config['interactive']

        self.load_kitti(self.data_folder, self.date,
                        self.drive, frames=self.frames)

        self.frame_iter = range(len(self.data_kitti.velo_files))

    def load_kitti(self, data_folder: str, date: str, drive: str, **kwargs):
        self.data_kitti = pykitti.raw(data_folder, date, drive, **kwargs)
        logging.info("Loading KITTI Dataset from dir: %r; date: %r, drive: %r",
                     self.data_folder, self.date, self.drive)
        self.load_projections(self.cam)
        self.fix_imu()
        self.img_n = IMG_WIDTH
        self.img_m = IMG_HEIGHT
        self.fix_missing_velo()

    def fix_missing_velo(self):
        """Fix issue with pykitti not handling missing velodyne data (files)
        """
        velo_frames = [int(Path(fpath).stem)
                       for fpath in self.data_kitti.velo_files]
        missing_frames = sorted(
            set(range(velo_frames[0], velo_frames[-1])) - set(velo_frames))
        # insert dummy frames, will throw exception when reading
        velo_files = self.data_kitti.velo_files.copy()
        for missing_frame in missing_frames:
            velo_files.insert(missing_frame, "BOGUS_FILE_WILL_THROW_EXCEPTION")
        self.data_kitti.velo_files = velo_files

    def load_projections(self, camN: str):
        """Loads projections for the camera of interest

        Arguments:
            camN {str} -- Camera cam0, cam1, cam2, cam3
        """
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
            reverse_yaw = rotz(oxts.packet.yaw).T
            reverse_yaw = transform_from_rot_trans(
                reverse_yaw, np.array([0, 0, 0]))
            T = reverse_yaw @ T
            new_oxts.append(OxtsData(oxts.packet, T))
        self.data_kitti.oxts = new_oxts

    def get_velo(self, frame_idx):
        """Gets veldoyne data of the frame index of interest

        Arguments:
            frame_idx {int} -- Frame index

        Returns:
            (ndarray, ndarray) -- Point cloud in the camN frame, corresponding intensity
        """
        pts3D_velo_unrectified = self.data_kitti.get_velo(frame_idx)
        pts3D_velo_unrectified = self.downsamle_pc(
            pts3D_velo_unrectified, self.pointcloud['downsample'])
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


    @staticmethod
    def normalize_data(data, scale=255, data_min=None, data_max=None):
        """ Normalize data """
        data_min = data_min if data_min else np.min(data)
        data_max = data_max if data_max else np.max(data)
        return ((data - data_min) / (data_max - data_min) * scale).astype(np.uint8)

    def load_frame(self, frame_idx: int):
        """Load frame from kitti

        Arguments:
            frame_idx {int} -- Frame index

        Returns:
            (img, pts2D, pts2D_color, pts3D) -- M X N ndarray image, projected lidar points, color for points, velodyne points
        """
        imgN = cv2.cvtColor(np.asarray(self.get_cam_fn(
            frame_idx)), cv2.COLOR_RGB2BGR)  # image
        pts3D_cam, intensity = self.get_velo(frame_idx)  # 3D points
        oxts = self.data_kitti.oxts[frame_idx]  # imu

        # project points
        pts2D_cam, idx = project_points(
            pts3D_cam, self.P_rect, self.img_m, self.img_n)

        # filter points outside of image
        pts3D_cam = pts3D_cam[:, idx].T[:, :3]
        intensity = intensity[idx]

        pose_cam = self.T_velo_imu @ oxts.T_w_imu
        logging.debug("Roll: %.3f; Pitch: %.3f; Yaw: %.3f", np.degrees(
            oxts.packet.roll), np.degrees(oxts.packet.pitch), np.degrees(oxts.packet.yaw))

        if self.pointcloud['color'] == 'intensity':
            color = intensity
            data_min = 0.0
            data_max = 1.0
        elif self.pointcloud['color'] == 'distance':
            distance = np.sqrt(
                pts3D_cam[:, 0] ** 2 + pts3D_cam[:, 1] ** 2 + pts3D_cam[:, 2] ** 2)
            color = distance
            data_min = -3
            data_max = 3
        else:
            z_height = -pts3D_cam[:, 1]
            color = z_height

        color = self.normalize_data(color)
        if self.pointcloud['outlier_removal']:
            t0 = time.time()
            # points3D_rot = points3D_rot[:400, :]
            mask = self.outlier_removal(pts3D_cam)
            pts3D_cam = pts3D_cam[~mask, :]
            t1 = time.time()
            t_point_filter = (t1 - t0) * 1000
        else:
            mask = np.zeros(color.shape, dtype=np.bool)

        return imgN, pts2D_cam, color, pts3D_cam, mask

    @staticmethod
    def outlier_removal(pc, stable_dist=0.1):
        """Remove outliers from a cpoint cloud
        Points on a sweeping beam scan should be near eachother
        """
        t0 = time.time()
        # shift point cloud
        pc_shift = np.roll(pc, -1, axis=0)
        # computer distance between points next to eachother on single scan
        diff = pc - pc_shift
        diff_norm = np.linalg.norm(diff, axis=1)
        # this will hold the mask of outlier points
        mask = np.zeros_like(diff_norm, dtype=np.bool)

        # TODO need outlier removal for average
        stable_dist_array = np.clip(diff_norm, 0.01, diff_norm)
        stable_dist_array = moving_average(
            stable_dist_array, n=10, pad_start=stable_dist)
        # print(stable_dist_array)
        stable_dist_array = np.clip(stable_dist_array, 0.02, stable_dist) * 1.5
        # print(stable_dist_array)
        # This is the patter we are looking for
        want_pattern = np.array(
            [True, True, False, False, True, True], dtype=np.bool)
        pc_pattern = diff_norm < stable_dist_array

        t1 = time.time()
        # print("Time: {}".format((t1-t0) * 1000))

        # TODO make fast stencil operator
        for i in range(4, pc_pattern.shape[0]-6):
            new_pattern = pc_pattern[i:i+6]
            if np.array_equal(want_pattern, new_pattern):
                mask[i+3] = True
                # print("Found one")
                # print(new_pattern)

        return mask

    @staticmethod
    def downsamle_pc(pc, ds=2):
        return pc[::ds, :]

    def run(self):
        if self.view_3D['active']:
            vis, pcd, all_polys = init_vis()

        # If configured to record video then prepare the writer
        if self.record['active']:
            record_frame_width = IMG_WIDTH
            record_frame_height = IMG_HEIGHT * 3 if self.record['stacked'] and self.view_3D['active'] else IMG_HEIGHT
            out_vid = cv2.VideoWriter(self.record['fpath'], cv2.VideoWriter_fourcc('M','J','P','G'), 10, (record_frame_width,record_frame_height))

        for frame_idx in self.frame_iter:
            # load image and point cloud
            try:
                img, pts2D_cam, color, pts3D_cam, mask = self.load_frame(
                    frame_idx)
            except Exception:
                logging.warn(
                    "Missing velodyne point cloud for frame, skipping...")
                continue
            # extract plane-like polygons
            points3D_rot, poly_rm, planes, obstacles, times = get_polygon(pts3D_cam, self.polylidar_kwargs, self.postprocess)
            # Get and print timing information
            (t_rotation, t_polylidar, t_polyfilter) = times
            logging.info("Frame idx: %d; Execution time(ms) - PC Rotation: %.1f; Polylidar: %.1f; Plane Filtering: %.1f",
                         frame_idx, t_rotation, t_polylidar, t_polyfilter)
            # print()

            # Write over 2D Image
            if self.view_image['show_pointcloud']:
                img = plot_points(img, pts2D_cam, color)
            if self.view_image['show_polygons']:
                plot_planes_and_obstacles(
                    planes, obstacles, self.P_rect, poly_rm, img, self.img_n, self.img_m)

            if self.view_image['active']:
                cv2.imshow(
                    'Image View - {}'.format(self.cam), img)
                if not self.view_3D['active']:
                    # Record frame for video
                    if self.record['active']:
                        out_vid.write(img)
                    
                    if self.interactive:
                        cv2.waitKey(10000)
                    else:
                        cv2.waitKey(1)
                    
            if self.view_3D['active']:
                cv2.waitKey(1)
                # Plot 3D Shapes in Open3D
                # colors = np.zeros_like(points3D_rot)
                # colors[mask] = [1, 0, 0]
                # pcd.colors = o3d.utility.Vector3dVector(colors)
                pcd.points = o3d.utility.Vector3dVector(points3D_rot)
                handle_shapes(planes, obstacles, all_polys)
                vis.update_geometry()
                vis.update_renderer()

                if frame_idx == 0:
                    set_initial_view(vis)

                # capture image of viewer
                if self.record['active']:
                    image_3D = (np.asarray(vis.capture_screen_float_buffer(True)) * 255).astype(np.uint8)
                    stacked_image = np.vstack((img, image_3D))
                    out_vid.write(stacked_image)
    
                # wait for user press a key on opencv image
                while(self.interactive):
                    vis.poll_events()
                    vis.update_renderer()
                    res = cv2.waitKey(33)
                    if res != -1:
                        break
