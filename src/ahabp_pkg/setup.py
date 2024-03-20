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
            'ahabp_node = ahabp_pkg.ahabp_node:main'
            #add more console scripts
            #[executable/node] = [package].[executable/node]:main
        ],
    },
)
