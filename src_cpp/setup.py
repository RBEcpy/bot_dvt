# setup.py (ở cùng cấp package.xml)
from setuptools import setup
import os
from glob import glob

package_name = 'src_cpp'  # phải trùng <name> trong package.xml

setup(
    name=package_name,
    version='0.0.1',
    packages=['nodes'],  # dùng thư mục nodes/ như 1 python package
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'save_model'), glob('save_model/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='DQN->TEB inference for Nav2 (ROS 2 Jazzy)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Cho phép: ros2 run src_cpp dqn_inference
            'dqn_inference = nodes.dqn_inference:main',
        ],
    },
)
