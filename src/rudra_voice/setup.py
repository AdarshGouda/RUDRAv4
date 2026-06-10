from glob import glob

from setuptools import find_packages, setup

package_name = 'rudra_voice'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'requests'],
    zip_safe=True,
    maintainer='Adarsh Gouda',
    maintainer_email='adhi.pesit@gmail.com',
    description='Safe local voice command interface for the RUDRA v4 rover.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'voice_node = rudra_voice.voice_node:main',
            'command_guard_node = rudra_voice.command_guard_node:main',
        ],
    },
)
