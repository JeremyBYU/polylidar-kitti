from setuptools import setup, find_packages
setup(
    name="kittiground",
    version="0.0.2",
    packages=['kittiground'],
    scripts=[],
    entry_points='''
        [console_scripts]
        kittiground=kittiground.cli:cli
    ''',

    install_requires=['numpy', 'pykitti', 'pyyaml', 'click', 'open3d', 'scipy', 'matplotlib', 'seaborn', 'pyximport'],

    # metadata to display on PyPI
    author="Jeremy Castagno",
    author_email="jdcasta@umich.edu",
    description="Polylidar and Kitti",
    license="MIT",
    keywords="kitti point cloud polygon",
    url="https://github.com/JeremyBYU/polylidar-kitti",   # project home page, if any
    project_urls={
        "Bug Tracker": "https://github.com/JeremyBYU/polylidar-kitti/issues",
    }
)