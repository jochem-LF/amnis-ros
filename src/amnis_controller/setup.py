import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amnis_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=[
        'setuptools',
        'websockets>=11.0,<12.0',
    ],
    zip_safe=True,
    maintainer='server',
    maintainer_email='server@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joystick_normalizer_node = amnis_controller.joystick_normalizer_node:main',
            'vehicle_controller_node = amnis_controller.vehicle_controller_node:main',
            'steer_controller_node = amnis_controller.steer_controller_node:main',
            'brake_controller_node = amnis_controller.brake_controller_node:main',
            'powertrain_controller_node = amnis_controller.powertrain_controller_node:main',
            'topic_aggregator_node = amnis_controller.topic_aggregator_node:main',
            'sensor_input_node = amnis_controller.sensor_input_node:main',
        ],
    },
)
