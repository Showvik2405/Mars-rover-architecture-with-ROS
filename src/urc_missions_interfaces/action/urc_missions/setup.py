from setuptools import setup
import os
from glob import glob

package_name = 'urc_missions'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Showvik Mondol Joy',
    maintainer_email='showvikmondol2405@gmail.com',
    description='URC missions with ROS2 nodes (Science, Delivery, Autonomous Navigation).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'coordinator = urc_missions.coordinator_node:main',
            'science = urc_missions.science_node:main',
            'delivery_server = urc_missions.delivery_action_server:main',
            'delivery_client = urc_missions.delivery_action_client:main',
            'autonav = urc_missions.autonav_node:main',
            'vision = urc_missions.vision_node:main',
        ],
    },
)

