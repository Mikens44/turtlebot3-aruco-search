from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_cube_search'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Visual ArUco cube search for TurtleBot3 and Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'aruco_detector = turtlebot3_cube_search.aruco_detector:main',
        'cube_localizer = turtlebot3_cube_search.cube_localizer:main',
        'cube_search_server = turtlebot3_cube_search.cube_search_server:main',
    ]},
)
