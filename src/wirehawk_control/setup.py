import os
from glob import glob
from setuptools import setup

package_name = 'wirehawk_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='Fast Kinematic Controller for Wirehawk CDPR',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cdpr_node = wirehawk_control.cdpr_node:main',
            'keyboard_teleop = wirehawk_control.keyboard_teleop:main',
        ],
    },
)