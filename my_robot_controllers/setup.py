from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fabian',
    maintainer_email='fabian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'p_controller = my_robot_controllers.p_controller:main',
            'pure_pursuit = my_robot_controllers.pure_pursuit_controller:main',
            'stanley_controller = my_robot_controllers.stanley_controller:main',
            'mpc_controller = my_robot_controllers.mpc_controller:main',
            'odom_plotter = my_robot_controllers.odom_plotter:main',
            
    ],
},
)
