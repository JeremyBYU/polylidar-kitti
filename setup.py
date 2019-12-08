from setuptools import setup, find_packages
setup(
    name="kittiground",
    version="0.0.1",
    packages=['grounddetector', 'kittiground'],
    scripts=[],

    install_requires=['numpy', 'pyrealsense2', 'pyyaml'],

    # metadata to display on PyPI
    author="Jeremy Castagno",
    author_email="jdcasta@umich.edu",
    description="Polylidar and Kitti",
    license="MIT",
    keywords="intel realsense point cloud polygon",
    url="https://github.com/JeremyBYU/polylidar-kitti",   # project home page, if any
    project_urls={
        "Bug Tracker": "https://github.com/JeremyBYU/polylidar-kitti/issues",
    }
)