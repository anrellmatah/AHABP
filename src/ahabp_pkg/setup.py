import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ahabp_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), #=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

##Start edit
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
##Stop edit

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anyell',
    maintainer_email='anyellmata@gmail.com',
    description='ROS2 package for ahabp project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'ahabp_node = ahabp_pkg.ahabp_node:main', # Don't need
            'ahabp_node_robot_upstart = ahabp_pkg.ahabp_node_robot_upstart:main',
            'ahabp_node_heartbeat = ahabp_pkg.ahabp_node_heartbeat:main',
            'ahabp_node_actuator_test = ahabp_pkg.ahabp_node_actuator_test:main',
            'ahabp_node_opencv = ahabp_pkg.ahabp_node_opencv:main',
            'ahabp_node_tracking = ahabp_pkg.ahabp_node_tracking:main',
            'ahabp_node_tracking_2 = ahabp_pkg.ahabp_node_tracking_2:main',
            'ahabp_node_offboard = ahabp_pkg.ahabp_node_offboard:main',
            'ahabp_node_ephem = ahabp_pkg.ahabp_node_ephem:main'
            #add more console scripts
            #[executable/node] = [package].[executable/node]:main
        ],
    },
)
