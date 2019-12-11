from pathlib import Path
import numpy as np

THIS_FILE = Path(__file__)
THIS_DIR = THIS_FILE.parent
DEFAULT_CONFIG_FILE = THIS_DIR / 'config' / 'default.yaml'

IMG_WIDTH = 1242
IMG_HEIGHT = 375

INTRINISCS = np.array([[649.51905284, 0.00000000, 620.50000000],
                       [0.00000000, 649.51905284, 374.50000000],
                       [0.00000000, 0.00000000, 1.00000000]])
EXTRINSICS = np.array([[0.99960128, 0.00806920, -0.02705864, -0.07041882],
                       [-0.01559983, -0.64094650, -0.76742702, 7.50137898],
                       [-0.02353566, 0.76754314, -0.64056507, 8.23519670],
                       [0.00000000, 0.00000000, 0.00000000, 1.00000000]])
