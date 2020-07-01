# Polylidar3D with KITTI Dataset - Ground/Obstacle Detection

This repository contains code and examples for using [Polylidar3D](https://github.com/JeremyBYU/polylidar) on the KITTI dataset to extract ground planes from point clouds. The example presented is for ground/obstacle detection with representation as polygons.  In this example the LiDAR is treated as an *unorganized* point cloud input to Polylidar3D. To learn more about Polylidar3D and its use cases for concave polygons extraction see it's [repository](https://github.com/JeremyBYU/polylidar).

<p align="center">
<img src="assets/media/2011_09_26_0005_stacked.gif" alt="Example ground and obstacle detection with Polylidar" style="max-width:100%; ">
</p>

The main components of the code are as follows:

1. Using `pykitti` to get camera and Velodyne LiDAR data.
2. Apply appropriate transformations of the pont cloud between vehicle body frame, Velodyne Frame, and Camera frame. This allows projecting the point cloud and polygons into the camera frame.
3. Use `Polylidar3D` to extract flat ground level surfaces and obstacles on said surfaces as polygons.
4. Perform polygon filtering, buffering (expansion/contraction), and simplification. This is done using Shapely/GEOS.
5. 2D Viewer - Project polygons onto camera image for display and verification.
6. 3D Viewer - Display 3D point clouds and polygons.

Please see disclaimers below before using Polylidar3D.

## Download Data

Download the data from [here](http://www.cvlibs.net/datasets/kitti/raw_data.php). I personally have only tested all drives that took place on 09/26/2011. These are quite large pieces of data.

After everything is extracted this is the format that I am expecting it to be in:

```txt
(base) ➜  data git:(polylidar3d) ✗ tree data -d    
.
data
├── 2011_09_26
│   ├── 2011_09_26_drive_0001_sync
│   │   ├── image_00
│   │   │   └── data
│   │   ├── image_01
│   │   │   └── data
│   │   ├── image_02
│   │   │   └── data
│   │   ├── image_03
│   │   │   └── data
│   │   ├── oxts
│   │   │   └── data
│   │   └── velodyne_points
│   │       └── data
│   ├── 2011_09_26_drive_0002_sync
│   │   ├── image_00
│   │   │   └── data
│   │   ├── image_01
│   │   │   └── data
│   │   ├── image_02
│   │   │   └── data
│   │   ├── image_03
│   │   │   └── data
│   │   ├── oxts
│   │   │   └── data
│   │   └── velodyne_points
│   │       └── data
.
.
.

```

## Installation

Please begin with first installing a python virtual environment.

1. Install [conda](https://conda.io/projects/conda/en/latest/) - [Why?](https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c)
2. `conda create --name polylidar3d python=3.6 && source activate polylidar3d` - Create new virtual python environment

There is one main dependencies which must be installed. Please `git clone` this repository in a separate directory in your workspace. You will need CMake to build this repository. Please note the installation section in the repo about building and *installing* `python` bindings. Please be sure that you have activated your newly created virtual environment when building this repository (`polylidar3d`).

1. [Polylidar3D](https://github.com/JeremyBYU/polylidar)

Once that is all done just install any dependencies needed in this repo.

1. `conda install -c conda-forge opencv shapely` - These packages give the most issue for binary dependencies for Windows users, hence why conda should handle them.
2. `pip install -e .` - Install any dependencies for this repository (`polylidar-kitti`).

## Running

A command line application should be installed after the `pip install` . Run as so:

```txt
$ kittiground run --help
Usage: kittiground run [OPTIONS] [INPUT]

  Run ground detector on KITTI Dataset.

  input is a file path to a yaml configuration file,
  Default=config/default.yaml

Options:
  --help  Show this message and exit.
$ kittiground run
```

The `default.yaml` contains all settings to run examples.  Comments should be on every parameter. 

## Disclaimers

Polylidar3D (for unorganized point cloud inputs) extracts all "flat" surfaces in the normal direction provided which must be aligned with the XY plane. The surfaces are represented as polygons. However nothing is truly flat and sensor measurements are noisy. Parameters guide polylidar in how "flat" a surface is. However these same paremeters that allow some noise and non-flatness also allow false-positives (e.g. the road connects into an elevated sidewalk). This code uses a lot of filtering **outside** of Polylidar3D to help reduce these false positives and make simpler polygons.  

## Notes

A small amount of (naive) pointcloud outlier filtering is performed __before__ the point cloud is sent to Polylidar3D. For speed up please install `Cython` -> `pip install Cython` .


<!-- ### Alternative Configs

```yaml
z_thresh: 0.15
norm_thresh_min: 0.98
bilateral_filter_normals(mesh, 5, 0.25, 0.25)
``` -->
