from setuptools import find_packages, setup
from glob import glob

package_name = 'my_robot_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wdy',
    maintainer_email='wdy@todo.todo',
    description='Simple robot goal package for MoveIt integration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_goal = my_robot_goal.my_robot_goal:main',
            'test_transform_sender = my_robot_goal.test_transform_sender:main'
        ],
    },
)
