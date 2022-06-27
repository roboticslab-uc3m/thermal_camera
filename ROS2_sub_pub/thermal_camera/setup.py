from setuptools import setup
import os
from glob import glob

package_name = 'thermal_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/camera_launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sheila',
    maintainer_email='sheilasr12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_sub_pub=thermal_camera.camera_sub_pub:main',
        ],
    },
)
