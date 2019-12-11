# Polylidar with KITTI Dataset - Ground/Obstacle Detection

This repository contains code and examples for using [Polylidar](https://github.com/JeremyBYU/polylidarv2) on the KITTI dataset to extact ground planes.  The example presented is for ground/obstacle detection with representation as polygons. To learn more about Polylidar and its use cases for concave polygons extraction see it's [repository](https://github.com/JeremyBYU/polylidarv2).

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



## Notes

Copy this to your clipboard (including braces).  Paste (Ctrl-V) into Open3D to get the correct view. I cant find a way to programatically do this. You can always manually adjust the view in the 3D Viewer.

```
{
	"class_name" : "ViewTrajectory",
	"interval" : 29,
	"is_loop" : false,
	"trajectory" : 
	[
		{
			"boundingbox_max" : [ 10.0, 20.0, 1.0 ],
			"boundingbox_min" : [ -10.0, -0.059999999999999998, -1.6299999999999999 ],
			"field_of_view" : 59.999999999999993,
			"front" : [ 0.023535657203938892, -0.76754313746794567, 0.6405650669259455 ],
			"lookat" : [ -0.027640775295681749, 11.82180199180485, -0.098162403753171285 ],
			"up" : [ 0.015599825759222748, 0.64094650338837789, 0.76742701622401466 ],
			"zoom" : 0.50000000127598909
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
```

Extriniscs:
```
[[0.99930320 0.02150350 -0.03050756 0.02921623]
 [-0.00203160 -0.78481335 -0.61972887 10.79278225]
 [-0.03726908 0.61935903 -0.78422281 10.95042993]
 [0.00000000 0.00000000 0.00000000 1.00000000]]
```


