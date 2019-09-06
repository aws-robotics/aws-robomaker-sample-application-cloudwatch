# Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
from setuptools import setup, find_packages

setup(
    name='aws_robomaker_simulation_common',
    version='2.0.0',
    packages=find_packages(),
    python_requires='>=3.5.0',
    install_requires=[
        'rospkg==1.1.7'
    ],
    zip_safe=True
)
