from glob import glob
import os
import sys

from setuptools import find_packages, setup

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

from kvn_orbslam3.prerequisites import warn_if_missing  # noqa: E402

_BUILD_COMMANDS = {'build', 'build_py', 'install', 'develop', 'bdist_wheel', 'egg_info'}
if _BUILD_COMMANDS.intersection(sys.argv[1:]):
    warn_if_missing(os.path.join(_here, 'scripts', 'install_prerequisites.sh'))

package_name = 'kvn_orbslam3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alessio Morale',
    maintainer_email='alessiomorale@gmail.com',
    description='ORB-SLAM3 RGB-D + IMU integration for the KVN rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'orbslam3_tf_bridge = kvn_orbslam3.tf_bridge_node:main',
            'orbslam3_occupancy_mapper = kvn_orbslam3.occupancy_mapper_node:main',
        ],
    },
)
