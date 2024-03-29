import os
from glob import glob
from setuptools import setup

package_name = 'pixelbot_interaction'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maure',
    maintainer_email='romain.maure@epfl.ch',
    description='Package taking care of the educative interaction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pixelbot_interaction_node = pixelbot_interaction.pixelbot_interaction_node:main'
        ],
    },
)
