from setuptools import setup, find_packages

package_name = 'tb3_remote_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/drive_forward.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Magnus',
    maintainer_email='magnusmeldgaard14@gmail.com',
    description='Control utilities for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_forward_10cm = tb3_remote_control.drive_forward_10cm:main',
            'file = tb3_remote_control.file:main',
            'streamAudio.py = tb3_remote_control.streamAudio:main',
            'audio_drive.py = tb3_remote_control.audio_drive:main',
            'streamAudioWrt.py = tb3_remote_control.streamAudioWrt:main',
        ],
    },
)
