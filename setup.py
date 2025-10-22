from setuptools import setup
import sys
import os
from glob import glob

venv_path = os.path.join(os.path.dirname(__file__), '.venv', 'bin', 'python')
if os.path.exists(venv_path):
    sys.executable = venv_path

package_name = 'warehouse_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ziga breznikar',
    maintainer_email='ziga.breznikar@student.um.si',
    description='Package for creating warehouse configurations and pickup scenarios via GUI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_empty_warehouse = warehouse_tools.create_empty_warehouse:main',
            'create_pickup_scenario = warehouse_tools.create_pickup_scenario:main'
        ],
    },
)
