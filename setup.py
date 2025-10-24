from setuptools import setup
import sys
import os
from glob import glob


#--Use local venv--#
# This is needed since I run pygame and pyyaml in venv. It can be commented out if you have them 
# globally
venv_path = os.path.join(os.path.dirname(__file__), '.venv', 'bin', 'python')
if os.path.exists(venv_path):
    sys.executable = venv_path

#--Name of ROS2 pkg
package_name = 'warehouse_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    #--data files--#
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), # This is a ROS2 pkg
        ('share/' + package_name, ['package.xml']), # find metadata such as dependecies
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), # install .launch.py files to ROS share dir
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # install .yaml files to ROS share dir
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ziga breznikar',
    maintainer_email='ziga.breznikar@student.um.si',
    description='Package for creating warehouse configurations and pickup scenarios via GUI.',
    license='MIT',
    tests_require=['pytest'],
    #--Entry points--#
    # To generate executable scripts
    entry_points={
        'console_scripts': [
            'create_empty_warehouse = warehouse_tools.create_empty_warehouse:main',
            'create_pickup_scenario = warehouse_tools.create_pickup_scenario:main'
        ],
    },
)
