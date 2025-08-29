from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'gecko_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'model'), glob('model/*')),
        (os.path.join('share', package_name, 'parameters'), glob('parameters/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cristianooo',
    maintainer_email='cristi_iorga02@yahoo.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'leg_teleop = gecko_robot.leg_teleop:main',
        'trajectory_leg_teleop = gecko_robot.trajectory_leg_teleop:main',
        'IK_solver = gecko_robot.IK_solver:main',
        'test_kdl_imports = gecko_robot.test_kdl_imports:main',
        ],
    },
)
