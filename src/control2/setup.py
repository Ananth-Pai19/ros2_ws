from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf*'))),
        # Install config files (if you have any)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Install RViz config files (if you have any)
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ananth',
    maintainer_email='paiananth0838@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "case2 = control2.autonomous_control_node_case2:main",
            "fullboi = control2.Full_Potential_Steering_Auto_ROS2:main",
            "cone_detector = control2.cone_detection:main",
            "case1 = control2.autonomous_control_code_case1:main",
        ],
    },
)
