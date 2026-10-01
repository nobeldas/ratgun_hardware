from glob import glob


from setuptools import find_packages, setup

package_name = 'target_tf_pkg'

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
            'apriltag_detection_status = '
            'target_tf_pkg.apriltag_detection_status:main',
            'coordinate_publisher = '
            'target_tf_pkg.coordinate_publisher:main',
            'coordinate_publisher_ordered = '
            'target_tf_pkg.coordinate_publisher_ordered:main',
            'fire_assist = target_tf_pkg.fire_assist:main',
        ],
    },
)


# data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/config', glob('config/*.yaml')),
#         ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
#     ],
