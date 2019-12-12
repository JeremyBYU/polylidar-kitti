
import numpy as np
import logging
import time
try:
    import cppyy
    cppyy.cppdef("""
    void pattern(uint8_t *src, uint8_t *dest, size_t src_size)  {
        for(size_t i = 2; i < src_size - 3; i++)
        {
            dest[i+1] = (src[i-2] > 0) && (src[i-1] > 0) && 
            (src[i] == 0) && (src[i+1] == 0) && 
            (src[i+2] > 0) && (src[i+3] > 0);
        }

    }   
    """)

    from cppyy.gbl import pattern as cpp_pattern

except Exception:
    logging.warn("Using slow python implementation")
    cpp_pattern = None

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

    # TODO need outlier removal for average
    stable_dist_array = np.clip(diff_norm, 0.01, diff_norm)
    stable_dist_array = moving_average(
        stable_dist_array, n=10, pad_start=stable_dist)
    stable_dist_array = np.clip(stable_dist_array, 0.02, stable_dist) * 1.5

    # This is the pattern we are looking for
    want_pattern = np.array(
        [True, True, False, False, True, True], dtype=np.bool)
    pc_pattern = diff_norm < stable_dist_array
    t1 = time.time()

    # this will hold the mask of outlier points
    mask = np.zeros_like(diff_norm, dtype=np.bool)
    mask = match_pattern(pc_pattern, mask, want_pattern, use_cpp=True)

    t2 = time.time()
    # print("Pattern Creating: {:.1f}; Pattern Matching: {:.1f}".format((t1-t0) * 1000, (t2-t1) * 1000))

    return mask

def python_pattern(pc_pattern, mask, want_pattern):
    for i in range(4, pc_pattern.shape[0]-6):
        new_pattern = pc_pattern[i:i+6]
        if np.array_equal(want_pattern, new_pattern):
            mask[i+3] = True

def match_pattern(pc_pattern, mask, want_pattern, use_cpp=True):
    if use_cpp and cpp_pattern:
        # print(pc_pattern, pc_pattern.dtype)
        # print(mask, mask.dtype)
        pc_pattern = pc_pattern.astype("uint8")
        mask = mask.astype("uint8")
        # print(pc_pattern, pc_pattern.dtype)
        # print(mask, mask.dtype)
        cpp_pattern(pc_pattern, mask, pc_pattern.shape[0])
        # print(np.count_nonzero(mask))
        return np.ma.make_mask(mask)
    else:
        python_pattern(pc_pattern, mask, want_pattern)
        # print(np.count_nonzero(mask))
        return mask
