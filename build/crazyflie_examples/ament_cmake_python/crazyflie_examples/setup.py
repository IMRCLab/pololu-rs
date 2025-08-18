from setuptools import find_packages
from setuptools import setup

setup(
    name='crazyflie_examples',
    version='1.0.3',
    packages=find_packages(
        include=('crazyflie_examples', 'crazyflie_examples.*')),
)
