from setuptools import find_packages, setup

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'ultralytics>=8.4.22',
    ],
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
            'demo_node = object_detection_pkg.demo_node:main',
            'ellipse_tracker = object_detection_pkg.ellipse_tracker:main',
            'color_ellipse_tracker = '
            'object_detection_pkg.color_ellipse_tracker:main',
            'cropped_yolo = object_detection_pkg.cropped_yolo:main',
            'webcam_yolo26 = object_detection_pkg.webcam_yolo26:main',
            'webcam_yolo_world = '
            'object_detection_pkg.webcam_yolo_world:main',
            'yolo_world_ratgun = '
            'object_detection_pkg.yolo_world_ratgun:main',
        ],
    },
)
