from setuptools import setup
import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'uav_inspection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='elena',
    maintainer_email='elena@todo.todo',
    description='UAV inspection package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
