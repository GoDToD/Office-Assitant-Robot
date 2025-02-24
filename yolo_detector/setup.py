from setuptools import setup
import os
from glob import glob

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='YOLOv8 object detection with ROS2 camera stream',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = yolo_detector.yolo_detector:main',  # This should match your script
        ],
    },
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)

