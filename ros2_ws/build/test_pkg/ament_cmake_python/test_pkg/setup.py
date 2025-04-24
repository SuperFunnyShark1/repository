from setuptools import find_packages
from setuptools import setup

setup(
    name='test_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('test_pkg', 'test_pkg.*')),
)
