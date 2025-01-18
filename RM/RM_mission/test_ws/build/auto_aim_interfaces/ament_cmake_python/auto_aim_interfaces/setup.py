from setuptools import find_packages
from setuptools import setup

setup(
    name='auto_aim_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('auto_aim_interfaces', 'auto_aim_interfaces.*')),
)
