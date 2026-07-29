from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'cam_drive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rnall',
    maintainer_email='devanneil2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cam_drive_control = cam_drive_control.cam_drive_control:main',
            'user_drive_control = cam_drive_control.user_drive_control:main',
            'auto_drive_control = cam_drive_control.auto_drive_control:main',
            'scout_camera_scanner = scan_pose_publisher.scout_camera_scanner:main',
            'demo_tf_publisher = scan_pose_publisher.demo_tf_publisher:main'
        ],
    },
)
