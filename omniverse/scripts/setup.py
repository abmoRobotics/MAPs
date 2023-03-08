"""Installation script for the 'omni.maps' python package."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from setuptools import find_packages, setup


# Minimum dependencies required prior to installation
INSTALL_REQUIRES = []

# Installation operation
setup(
    name="mapper",
    author="Anton Bjørndahl Mortensen",
    version="1.0.0",
    description="Environment for materials acceleration platforms",
    keywords=["robotics", "materials acceleration platforms"],
    include_package_data=True,
    install_requires=INSTALL_REQUIRES,
    packages=find_packages(where=['omni']),#where="./package"),
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.7, 3.8",
    ],
    zip_safe=False,
)
