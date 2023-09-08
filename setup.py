import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'tswr_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='konrad.jedrzejewski@student.put.poznan.pl',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cem_controller = tswr_controllers.cem_controller:main',
            'icem_controller = tswr_controllers.icem_controller:main',
            'validate_topic_sent.py = tswr_controllers.validate_topic_sent:main',
        ],
    },
)
