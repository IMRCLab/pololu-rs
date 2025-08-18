from setuptools import find_packages
from setuptools import setup

setup(
    name='motion_capture_tracking_interfaces',
    version='1.0.6',
    packages=find_packages(
        include=('motion_capture_tracking_interfaces', 'motion_capture_tracking_interfaces.*')),
)
