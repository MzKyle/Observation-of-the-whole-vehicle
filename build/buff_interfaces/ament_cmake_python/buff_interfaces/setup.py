from setuptools import find_packages
from setuptools import setup

setup(
    name='buff_interfaces',
    version='1.0.0',
    packages=find_packages(
        include=('buff_interfaces', 'buff_interfaces.*')),
)
