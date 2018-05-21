## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['follow_joint_trajectory_action_adapter', 'arm_composites_manufacturing_controller', 'arm_composites_manufacturing_controller_adapters'],
    package_dir={'': 'src'})

setup(**setup_args)
