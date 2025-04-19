from setuptools import find_packages
from setuptools import setup

setup(
    name='my_first_pkg',
    version='0.0.0',
    packages=find_packages(
        include=('my_first_pkg', 'my_first_pkg.*')),
)
