from setuptools import setup
import os
from glob import glob

package_name = 'orchestrator'
module_utils = 'utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, module_utils],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton, Mads',
    maintainer_email='abmo19@student.aau.dk, mrma19@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shuttle = orchestrator.shuttle:main',
            'gui = orchestrator.pyside_gui:main',
            'manipulator = orchestrator.manipulator:main',
            'task_planner = orchestrator.task_planner:main',
            'spawn_manager = orchestrator.spawn_manager:main',
        ],
    },
)
