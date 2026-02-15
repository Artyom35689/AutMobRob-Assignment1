from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghadeer',
    maintainer_email='ghadeer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = my_robot_bringup.keyboard_control:main',
            'odom_listener = my_robot_bringup.odom_listener:main',
        ],
    },
)
