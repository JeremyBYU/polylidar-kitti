# distutils: language = c
# cython: language_level=2

import numpy as np
cimport numpy as np
cimport cython

DTYPE = np.uint8

ctypedef np.uint8_t DTYPE_t

@cython.boundscheck(False) # turn off bounds-checking for entire function
@cython.wraparound(False)  # turn off negative index wrapping for entire function
def c_pattern(np.ndarray[DTYPE_t, ndim=1] src, np.ndarray[DTYPE_t, ndim=1] dest):
    assert src.dtype == DTYPE and dest.dtype == DTYPE
    assert src.shape[0] == dest.shape[0]

    cdef int src_size = src.shape[0]
    cdef int i

    cdef DTYPE_t value
    # Looking for pattern True, True, False, False, True, True
    for i in range(2, src_size - 3):
        dest[i+1] = (src[i-2] > 0) and (src[i-1] > 0) and \
        (src[i] == 0) and (src[i+1] == 0) and \
        (src[i+2] > 0) and (src[i+3] > 0)