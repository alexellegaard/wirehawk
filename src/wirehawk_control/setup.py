from setuptools import find_packages, setup

package_name = 'wirehawk_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'motor_driver = wirehawk_control.motor_driver:main',
        'joy_controller = wirehawk_control.joy_controller:main',
        'trajectory_controller = wirehawk_control.trajectory_controller:main'
    ],
},
)
