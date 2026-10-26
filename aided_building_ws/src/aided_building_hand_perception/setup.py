from glob import glob

from setuptools import setup

package_name = 'aided_building_hand_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aided Building Team',
    maintainer_email='team@example.com',
    description='Camera-based hand detection relative to the Franka arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_detector = aided_building_hand_perception.hand_detector_node:main',
            'workbench_calibration = '
            'aided_building_hand_perception.workbench_calibration:main',
            'generate_marker = aided_building_hand_perception.generate_marker:main',
        ],
    },
)
