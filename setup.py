import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'time_checker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='isseitezuka',
    maintainer_email='issei.tezuka@tier4.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'goal_line_time_measurer = time_checker.goal_line_time_measurer:main',
          'start_to_goal_line_time_measurer = time_checker.start_to_goal_line_time_measurer:main',
          'polygon_pass_opposite_line_time_measurer = time_checker.polygon_pass_opposite_line_time_measurer:main',
        ],
    },
)
