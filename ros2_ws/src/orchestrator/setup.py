from setuptools import setup
import os
from glob import glob

package_name = 'orchestrator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu20',
    maintainer_email='mrma19@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shuttle = orchestrator.shuttle:main'
            #'manipulator = orchestrator.manipulator:main'
            #'task_planner = orchestrator.task_planner:main'
        ],
    },
)
