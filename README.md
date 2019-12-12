# Polylidar with KITTI Dataset - Ground/Obstacle Detection

This repository contains code and examples for using [Polylidar](https://github.com/JeremyBYU/polylidarv2) on the KITTI dataset to extact ground planes from point clouds.  The example presented is for ground/obstacle detection with representation as polygons. To learn more about Polylidar and its use cases for concave polygons extraction see it's [repository](https://github.com/JeremyBYU/polylidarv2).

<p align="center">
<img src="assets/media/2011_09_26_0005_stacked.gif" alt="Example ground and obstacle detection with Polylidar" style="max-width:100%;">
</p>

The main components of the code are as follows:
1. Using `pykitti` to get camera and velodyne lidar data.
2. Apply appropriate transfomations so that the camera and lidar are in the same coordinate system.
3. Use `polylidar` to extract ALL flat surfaces and obstacles on said surfaces as polygons.
4. Perform polygon filtering, buffering (expansion/contraction), and simplification.
5. 2D Viewer - Project polygons onto camera image for display and verification.
6. 3D Viewer - Display 3D point clouds and polygons.

Please see disclaimers below before using Polylidar.

## Installation

1. Install [conda](https://conda.io/projects/conda/en/latest/) - [Why?](https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c)
2. `conda create --name kitti python=3.7 && source activate kitti` - Create new virtual python environment
3. `git clone --recurse-submodules https://github.com/JeremyBYU/polylidar-kitti.git && cd polylidar-kitti`
4. `conda install -c conda-forge opencv shapely` - These packages gives an issue for binary dependencies for Windows users, hence why conda should handle it. Use pip if on linux.
5. `cd thirdparty/polylidar && pip install -e . && cd ../..` - Install polylidar manually because it is not on PyPi.
6. `pip install -e .` - Install any dependencies for this repository (kittiground).


## Running

A command line applicaton should be installed after the `pip install`. Run as so:

```
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

Polylidar extracts all "flat" surfaces that are connected as polygons. However nothing is truly flat and sensor measurements are noisy. Parameters guide polylidar in how "flat" a surface is. However these same paremeters that allow some noise and non-flatness also allow false-positives (e.g. the road connects into a sidewalk). This code uses a lot of filtering **outside** of polylidar to help reduce these false positives and make simpler polygons.  


