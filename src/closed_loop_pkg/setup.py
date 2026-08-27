from glob import glob

from setuptools import find_packages, setup

package_name = 'closed_loop_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scorpion',
    maintainer_email='nobel.das16z@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_obj_coordinate = closed_loop_pkg.base_obj_coordinate:main',
            'base_gun_transformation = closed_loop_pkg.base_gun_transformaton:main',
            'pan_tilt_pub = closed_loop_pkg.pan_tilt_pub:main',
            'turrent_command = closed_loop_pkg.turrent_command:main',
        ],
    },
)
