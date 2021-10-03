#!/usr/bin/env python2

from distutils.core import setup

setup(
    name="crsf_drv",
    version="1.0",
    description="Python Distribution Utilities",
    author="Alessio Morale",
    author_email="alessiomorale@gmail.com",
    packages=["crsf_drv"],
    package_dir={"": "crsf_drv"},
)

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['crsf_drv'],
    scripts=['scripts/crsf_drv_node.py'],
    package_dir={'': '.'}
)

setup(**d)
