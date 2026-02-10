from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wmr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'deps/wmr-simulator/scripts'), glob('deps/wmr-simulator/scripts/*.py')),
    ],
    entry_points={
        'console_scripts': [
            'wmr_controller_node = wmr_controller.wmr_controller_node:main',
            'ann_cmcgs_node = wmr_controller.ann_cmcgs_node:main',
            'reference_publisher_node = wmr_controller.reference_publisher_example:main',
            'timing_monitor = wmr_controller.timing_monitor:main'
        ],
    },
)
