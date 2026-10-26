from setuptools import setup

package_name = 'aided_building_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aided Building Team',
    maintainer_email='team@example.com',
    description='Simple ROS 2 command interface for the Franka arm',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'comm_node = aided_building_comm.comm_node:main',
            'clock_filter = aided_building_comm.clock_filter:main',
        ],
    },
)
