from setuptools import find_packages, setup

package_name = 'simple_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_mapper.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='最简单的传感器数据到关节角度映射算法Demo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_mapper_node = simple_demo.simple_mapper_node:main',
        ],
    },
)

