from setuptools import find_packages, setup

package_name = 'gsplat_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Publishes latched SplatArray messages loaded from a PLY file.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ply_splat_publisher = gsplat_publisher.ply_splat_publisher:main',
        ],
    },
)
