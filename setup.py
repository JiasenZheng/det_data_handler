from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['det_data_handler'],
    package_dir={'': 'python'},
    scripts=['scripts/data_collection', 'scripts/handle_coco', 'scripts/collect_lidar_cam'],
)

setup(**d)
