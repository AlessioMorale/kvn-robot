from glob import glob

from setuptools import find_packages, setup

package_name = 'kvn_odom_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alessio Morale',
    maintainer_email='alessiomorale@gmail.com',
    description='Automatic wheel odometry calibration for the KVN rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_calibration = kvn_odom_calibration.odom_calibration_node:main',
        ],
    },
)
