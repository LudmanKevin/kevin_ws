from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'onrobot_eye_integration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnl',
    maintainer_email='arnl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onrobot_eye_node = onrobot_eye_integration.onrobot_eye_node:main',
            'onrobot_pickup = onrobot_eye_integration.onrobot_pickup:main',
        ],
    },
)
