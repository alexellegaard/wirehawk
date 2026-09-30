from setuptools import setup

package_name = 'wirehawk_gazebo_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='Sim bridge: counts <-> Gazebo cable lengths (stand-in for the EtherCAT bridge)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_bridge = wirehawk_gazebo_bridge.sim_bridge_node:main',
        ],
    },
)
