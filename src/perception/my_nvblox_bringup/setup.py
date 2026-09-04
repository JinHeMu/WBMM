from glob import glob
import os

from setuptools import setup

package_name = 'my_nvblox_bringup'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 安装 launch 文件
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.sh')),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='a',
    maintainer_email='a@todo.todo',
    description='MuJoCo and D455 bringup for nvblox 3D ESDF mapping',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'esdf_visualizer = my_nvblox_bringup.esdf_visualizer:main',
            'nvblox_map_exporter = '
            'my_nvblox_bringup.nvblox_map_exporter:main',
            'map_snapshot_saver = '
            'my_nvblox_bringup.map_snapshot_saver:main',
        ],
    },
)
