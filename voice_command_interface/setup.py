from setuptools import setup
import os
from glob import glob

package_name = 'voice_command_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'openai',
        'speechrecognition'
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 package to capture voice input and parse intent via Azure OpenAI.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_listener = voice_command_interface.voice_listener:main',
        ],
    },
)

