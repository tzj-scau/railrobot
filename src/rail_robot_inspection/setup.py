from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'rail_robot_inspection'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    package_data={
        package_name: ['*.dll', '*.so', '*.pyd', '*.lib']
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'orbbeccamera'), glob('orbbeccamera/*')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sensor_msgs',
        'cv_bridge',
        'image_transport',
        'rail_robot_inspection_msgs',
        'pytest',
        'pytest-cov'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Rail robot inspection package for ROS2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'rail_control_node = rail_robot_inspection.rail_control_node:main',
            'orbbec_camera_node = rail_robot_inspection.orbbec_camera_node:main',
            'thermal_camera_node = rail_robot_inspection.thermal_camera_node:main',
            'serial_bridge_node = rail_robot_inspection.serial_bridge_node:main',
        ],
    },
    python_requires='>=3.6',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
)