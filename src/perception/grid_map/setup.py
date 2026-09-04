from glob import glob

from setuptools import find_packages, setup

package_name = 'grid_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/maps',
            glob('grid_map/maps/*.npz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='490754775@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mjcf_to_esdf = grid_map.mjcf_to_esdf:main',
            'esdf_rviz_publisher = grid_map.esdf_rviz_node:main',
            'annotate_board = grid_map.annotate_board:main',
            'board_marker = grid_map.board_marker:main',
        ],
    },
)
