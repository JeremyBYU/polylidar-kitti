import time
import logging
from copy import deepcopy

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import open3d as o3d

from fastga import GaussianAccumulatorS2, MatX3d, IcoCharts
from fastga.peak_and_cluster import find_peaks_from_ico_charts
from fastga.o3d_util import get_arrow, get_pc_all_peaks, get_arrow_normals

from kittiground.kittiground.open3d_util import assign_vertex_colors, create_open_3d_mesh, get_colors

default_level=5

ga_default = GaussianAccumulatorS2(level=default_level)  # Fast Gaussian Accumulator
ico_chart_default = IcoCharts(level=default_level)  # Used for unwrapping S2 to Image for peak detection


def down_sample_normals(triangle_normals, down_sample_fraction=0.12, min_samples=10000, flip_normals=False, **kwargs):
    num_normals = triangle_normals.shape[0]
    to_sample = int(down_sample_fraction * num_normals)
    to_sample = max(min([num_normals, min_samples]), to_sample)
    ds_step = int(num_normals / to_sample)
    triangle_normals_ds = np.ascontiguousarray(triangle_normals[:num_normals:ds_step, :])
    if flip_normals:
        triangle_normals_ds = triangle_normals_ds * -1.0
    return triangle_normals_ds

def extract_all_dominant_plane_normals(tri_mesh, level=default_level, with_o3d=True, ga_=ga_default, ico_chart_=ico_chart_default, **kwargs):

    # Reuse objects if provided
    if ga_ is not None:
        ga = ga_
    else:
        ga = GaussianAccumulatorS2(level=level)

    if ico_chart_ is not None:
        ico_chart = ico_chart_
    else:
        ico_chart = IcoCharts(level=level)

    triangle_normals = np.asarray(tri_mesh.triangle_normals)
    triangle_normals_ds = down_sample_normals(triangle_normals, **kwargs)

    triangle_normals_ds_mat = MatX3d(triangle_normals_ds)
    t1 = time.perf_counter()
    ga.integrate(triangle_normals_ds_mat)
    t2 = time.perf_counter()

    logging.info("Gaussian Accumulator - Normals Sampled: %d; Took (ms): %.2f",
                triangle_normals_ds.shape[0], (t2 - t1) * 1000)

    avg_peaks, pcd_all_peaks, arrow_avg_peaks, timings_dict = get_image_peaks(
        ico_chart, ga, level=level, with_o3d=with_o3d, **kwargs)

    gaussian_normals = np.asarray(ga.get_bucket_normals())
    accumulator_counts = np.asarray(ga.get_normalized_bucket_counts())

    # Create Open3D structures for visualization
    if with_o3d:
        # Visualize the Sphere
        refined_icosahedron_mesh = create_open_3d_mesh(np.asarray(ga.mesh.triangles), np.asarray(ga.mesh.vertices))
        color_counts = get_colors(accumulator_counts)[:, :3]
        colored_icosahedron = assign_vertex_colors(refined_icosahedron_mesh, color_counts)
    else:
        colored_icosahedron = None

    elapsed_time_fastga = (t2 - t1) * 1000
    elapsed_time_peak = timings_dict['fastga_peak']
    elapsed_time_total = elapsed_time_fastga + elapsed_time_peak

    timings = dict(fastga_total=elapsed_time_total, fastga_integrate=elapsed_time_fastga, fastga_peak=elapsed_time_peak)

    # Must clear the gaussian accumulator to prepare for next integration of surface normals
    ga.clear_count()

    return avg_peaks, pcd_all_peaks, arrow_avg_peaks, colored_icosahedron, timings


def get_image_peaks(ico_chart, ga, level=2, with_o3d=True,
                    find_peaks_kwargs=dict(threshold_abs=2, min_distance=1, exclude_border=False, indices=False),
                    cluster_kwargs=dict(t=0.10, criterion='distance'),
                    average_filter=dict(min_total_weight=0.01),
                    **kwargs):

    normalized_bucket_counts_by_vertex = ga.get_normalized_bucket_counts_by_vertex(True)

    ico_chart.fill_image(normalized_bucket_counts_by_vertex)
    # image = np.asarray(ico_chart.image)
    # plt.imshow(image)
    # plt.show()
    find_peaks_kwargs = dict(threshold_abs=20, min_distance=1, exclude_border=False, indices=False)
    cluster_kwargs = dict(t=0.30, criterion='distance')
    # average_filter = dict(min_total_weight=0.01)

    t1 = time.perf_counter()
    peaks, clusters, avg_peaks, avg_weights = find_peaks_from_ico_charts(ico_chart, np.asarray(
        normalized_bucket_counts_by_vertex), find_peaks_kwargs, cluster_kwargs, average_filter)
    t2 = time.perf_counter()
    gaussian_normals_sorted = np.asarray(ico_chart.sphere_mesh.vertices)
    # Create Open3D structures for visualization
    if with_o3d:
        pcd_all_peaks = get_pc_all_peaks(peaks, clusters, gaussian_normals_sorted)
        arrow_avg_peaks = get_arrow_normals(avg_peaks, avg_weights)
    else:
        pcd_all_peaks = None
        arrow_avg_peaks = None

    elapsed_time = (t2 - t1) * 1000
    timings = dict(fastga_peak=elapsed_time)

    logging.info("Peak Detection - Took (ms): %.2f", (t2 - t1) * 1000)

    return avg_peaks, pcd_all_peaks, arrow_avg_peaks, timings