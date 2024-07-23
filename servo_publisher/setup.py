from setuptools import setup
import os
from glob import glob

package_name = 'servo_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['servo_publisher.rotation'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Farrukh',
    maintainer_email='farrukhajaz1@gmail.com',
    description='A ROS 2 package for rotating a servo motor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rotation = servo_publisher.rotation:main',
        ],
    },
)

