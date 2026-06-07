from glob import glob

from setuptools import find_packages, setup

package_name = 'rudra_base_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
        (
            'share/' + package_name + '/firmware/uno_ps2_plain_serial',
            glob('firmware/uno_ps2_plain_serial/*'),
        ),
        (
            'share/' + package_name + '/firmware/teensy_sabertooth_serial_controller',
            glob('firmware/teensy_sabertooth_serial_controller/*'),
        ),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Adarsh Gouda',
    maintainer_email='adhi.pesit@gmail.com',
    description='ROS2 bridge for RUDRA PS2 Uno input and Teensy/Sabertooth base control.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'ps2_uno_to_teensy = rudra_base_bridge.ps2_uno_to_teensy:main',
            'cmd_vel_to_teensy = rudra_base_bridge.cmd_vel_to_teensy:main',
            'list_serial_ports = rudra_base_bridge.serial_port_list:main',
        ],
    },
)
