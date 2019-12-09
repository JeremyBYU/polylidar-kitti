import pykitti


class KittiGround(object):
    def __init__(self, config):
        super().__init__()
        self.data_folder: str = config['data_folder']
        self.date: str = config['data']
        self.drive: str = config['drive']
        self.frames = config['frames']
        self.view_image = config['view_image']
        self.view_3D = config['view_3D']
        self.polylidar_kwargs = config['polygon']['polylidar']
        self.postprocess = config['postprocess']

        self.load_kitti(self.data_folder, self.date,
                        self.drive, frames=self.frames)
        self.frame_iter = range(len(self.data_kitti.velo_files))

    def load_kitti(self, data_folder: str, date: str, drive: str, **kwargs):
        self.data_kitti = pykitti.raw(data_folder, date, drive, **kwargs)

    def load_frame(self, frame_index):
        
