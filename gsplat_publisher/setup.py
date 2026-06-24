import glob
import os

from setuptools import find_packages, setup

package_name = 'gsplat_publisher'

splats_dir = os.path.join(os.path.dirname(__file__), '..', 'splats')
ply_files = glob.glob(os.path.join(splats_dir, '*.ply'))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'splats'), ply_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Publishes Gaussian splat arrays and chunked splat streams from PLY files.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blob_snapshot_publisher = gsplat_publisher.blob_snapshot_publisher:main',
            'ply_splat_publisher = gsplat_publisher.ply_splat_publisher:main',
            'tile_demo_publisher = gsplat_publisher.tile_demo_publisher:main',
        ],
    },
)
