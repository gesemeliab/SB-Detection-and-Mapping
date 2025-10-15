#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Setup script for plotting_points package.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['plotting_points'],
    package_dir={'': 'src'},
)

setup(**setup_args)
