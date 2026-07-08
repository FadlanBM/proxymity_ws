from setuptools import find_packages, setup

package_name = 'proxymity'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['proxymity/config/params.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/proxymity_launch.py', 'launch/proxymity_controller_launch.py', 'launch/green_light_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fadlan',
    maintainer_email='fadlan@heroes-jaya.com',
    description='ROS 2 package for proximity sensor reading on Jetson Orin Nano via GPIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proxymity_node = proxymity.sensor_node:main',
            'proxymity_controller_node = proxymity.sensor_controller_node:main',
            'green_light_node = proxymity.green_light_node:main',
        ],
    },
)
