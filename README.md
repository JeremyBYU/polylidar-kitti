# Polylidar with Kitti Dataset - Ground/Obstacle Detection

This repository contains code and examples for using [Polylidar](https://github.com/JeremyBYU/polylidarv2) on the KITTI dataset to extact ground planes.  The example presented is for ground/obstacle detection with representation as polygons. To learn more about Polylidar and its use cases for concave polygons extraction see it's [repository](https://github.com/JeremyBYU/polylidarv2).

<!-- The main components of the code are as follows:
1. Using `pyrealsense2` to interface with the a D435 sensor.
2. Apply filtering to generated depth images (spatial,temporal, etc.).  
3. Generate a point cloud from filtered depth image.
4. Find ground normal and rotate point cloud to align its z-axis with ground normal.
5. Use `polylidar` to extract flat surfaces and obstacles as polygons
6. Perform polygon filtering and buffering.
7. Project polygons onto image for display and verification -->

Please see disclaimers below before using Polylidar with an Intel RealSense camera for your application.

## Installation

1. Install [conda](https://conda.io/projects/conda/en/latest/) - [Why?](https://medium.freecodecamp.org/why-you-need-python-environments-and-how-to-manage-them-with-conda-85f155f4353c)
2. `conda create --name kitti python=3.6 && source activate kitti` - Create new virtual python environment
3. `git clone --recurse-submodules https://github.com/JeremyBYU/polylidar-realsense.git && cd polylidar-realsense`
4. `conda install -c conda-forge opencv shapely` - These packages give the most issue for binary dependencies for Windows users, hence why conda should handle them. Use pip if on linux.
5. `cd thirdparty/polylidar && pip install -e . && cd ../..` - Install polylidar manually because it is not on PyPi.
6. `pip install -e .` - Install any dependencies for this repository (groundetector).


## Running


## Disclaimers

## Notes

Copy this to your clipboard.  Paste (Ctrl-V) into Open3D to get the correct view

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
			"field_of_view" : 60.0,
			"front" : [ -0.002797466300819589, -0.93017941771777879, 0.36709457233322029 ],
			"lookat" : [ 0.0, 9.9700000000000006, -0.31499999999999995 ],
			"up" : [ 0.015936993373838547, 0.36700791577643127, 0.93008128784512278 ],
			"zoom" : 0.45999999999999974
		}
	],
	"version_major" : 1,
	"version_minor" : 0
}
```


