#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Setup script for sb_detection package.

This allows the package to be installed as a Python module,
making imports cleaner and enabling proper testing.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['sb_detection'],
    package_dir={'': 'src'},
    install_requires=[
        'numpy',
        'opencv-python',
        'tensorflow',
        'keras',
    ],
)

setup(**setup_args)
