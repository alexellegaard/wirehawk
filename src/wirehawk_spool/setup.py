import os
from glob import glob
from setuptools import setup

package_name = 'wirehawk_spool'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='Shared winch/spool model: encoder-counts <-> cable-length conversion',
    license='Apache-2.0',
)
