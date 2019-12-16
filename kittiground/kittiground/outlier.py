
import numpy as np
import logging
import time
import sys
from scipy.signal import medfilt
try:
    # import cppyy
    # cppyy.cppdef("""
    # void pattern(uint8_t *src, uint8_t *dest, size_t src_size)  {
    #     for(size_t i = 2; i < src_size - 3; i++)
    #     {
    #         dest[i+1] = (src[i-2] > 0) && (src[i-1] > 0) && 
    #         (src[i] == 0) && (src[i+1] == 0) && 
    #         (src[i+2] > 0) && (src[i+3] > 0);
    #     }

    # }   
    # """)

    # from cppyy.gbl import pattern as cpp_pattern
    import pyximport
    pyximport.install(setup_args={'include_dirs': np.get_include()})
    from kittiground.kittiground.outlierfast import c_pattern_6, c_pattern_2


except Exception:
    logging.warn("Using slow python implementation")
    c_pattern = None

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

def outlier_removal(pc, single_beam_dist=0.1, single_beam_multiplier=1.5, max_dist=15.0 ):
    """Remove outliers from a point cloud
    Points on a **single** sweeping beam scan should be near eachother (single_beam_dist) when on a plane
    Points on a **single* sweeeping beam should never exceed the max_dist. If they do they are most likely outliers 
    """
    t0 = time.time()
    # shift point cloud
    pc_shift = np.roll(pc, -1, axis=0)
    # computer distance between points next to eachother on single scan
    diff = pc - pc_shift
    diff_norm = np.linalg.norm(diff, axis=1)

    # For planar noise, when a single beam is scanning a plane
    stable_dist_array = medfilt(diff_norm, kernel_size=9)
    stable_dist_array = np.clip(stable_dist_array, 0.01, single_beam_dist) * single_beam_multiplier
    pc_pattern_plane = diff_norm < stable_dist_array
    # For planar noise introduced by a completely different sensor beam
    # This is very unstable
    pc_pattern_max = diff_norm < max_dist

    # this will hold the mask of outlier points
    mask = np.zeros_like(diff_norm, dtype=np.bool)
    mask = match_pattern(pc_pattern_plane, pc_pattern_max, mask, use_compiled=True)

    t2 = time.time()
    # print("Pattern Creating: {:.1f}; Pattern Matching: {:.1f}".format((t1-t0) * 1000, (t2-t1) * 1000))

    return mask

def python_pattern(pc_pattern, mask, want_pattern):
    length_want_pattern = want_pattern.shape[0]
    jump_index = 3 if length_want_pattern == 6 else 1
    for i in range(length_want_pattern - 2, pc_pattern.shape[0]-length_want_pattern):
        new_pattern = pc_pattern[i:i+length_want_pattern]
        if np.array_equal(want_pattern, new_pattern):
            mask[i+jump_index] = True

def match_pattern(pc_pattern_plane, pc_pattern_max, mask, use_compiled=True):
    if use_compiled and c_pattern_6:

        pc_pattern_plane = pc_pattern_plane.astype("uint8")
        pc_pattern_max = pc_pattern_max.astype("uint8")
        mask = mask.astype("uint8")

        # cpp_pattern(pc_pattern, mask, pc_pattern.shape[0])
        c_pattern_6(pc_pattern_plane, mask) # plane noise
        c_pattern_2(pc_pattern_max, mask) # max dist
        # print(np.count_nonzero(mask))
        return np.ma.make_mask(mask)
    else:
        want_pattern_plane = np.array([True, True, False, False, True, True], dtype=np.bool)
        want_pattern_max = np.array([False, False], dtype=np.bool)
        python_pattern(pc_pattern_plane, mask, want_pattern_plane)
        python_pattern(pc_pattern_max, mask, want_pattern_max)
        # print(np.count_nonzero(mask))
        return mask
