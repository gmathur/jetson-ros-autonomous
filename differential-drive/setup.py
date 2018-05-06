#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['differential_drive'],
    scripts=['scripts/diff_tf.py', 'scripts/pid_velocity.py',
        'scripts/wheel_scaler.py', 'scripts/twist_to_motors.py', 'scripts/virtual_joystick.poy'],
    package_dir={'': 'scripts'},
    install_requires=['rospkg'],
    )

setup(**d)
