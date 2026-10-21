from setuptools import find_packages, setup

package_name = 'gen3_target_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'ik_test = gen3_target_control.ik_test:main',
        'target_controller = gen3_target_control.target_controller:main',
        'xyz_mapper = gen3_target_control.xyz_mapper:main',
        'target_visualizer = gen3_target_control.target_visualizer:main',
        'pose_bridge = gen3_target_control.pose_bridge:main',
        ],
    },
)
