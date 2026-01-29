from setuptools import find_packages, setup

package_name = 'wmr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'wmr_controller_node = wmr_controller.wmr_controller_node:main',
            'reference_publisher_node = wmr_controller.reference_publisher_example:main',
            'timing_monitor = wmr_controller.timing_monitor:main'
        ],
    },
)
