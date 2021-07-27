from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # packages=['fast_task_planner'],
    packages=find_packages(),
    # scripts=['bin/myscript'],
    package_dir={'': 'src'}
)

setup(**d)