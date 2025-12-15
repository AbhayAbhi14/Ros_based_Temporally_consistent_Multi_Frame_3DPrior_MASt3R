from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mono_reconstruct'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhaypayadi',
    maintainer_email='abhaypayyadi@gmnail.com',
    description='Monocular reconstruction camera node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = mono_reconstruct.camera_node:main',
            'mast3r_node = mono_reconstruct.mast3r_node:main',
            'camera_tf_broadcaster = mono_reconstruct.camera_tf_broadcaster:main',
            'temporal_fusion_node = mono_reconstruct.temporal_fusion_node:main',
            'temporal_fusion_fixed = mono_reconstruct.temporal_fusion_fixed:main',
            'overlay_projector_tf = mono_reconstruct.overlay_projector_tf:main',
            'depth_compare = mono_reconstruct.depth_compare:main',
        ],
    },
)
