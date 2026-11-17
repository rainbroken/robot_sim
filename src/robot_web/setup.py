from glob import glob
from setuptools import find_packages, setup

setup(
    name='robot_web', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_web']),
        ('share/robot_web', ['package.xml']),
        ('share/robot_web/launch', glob('launch/*.launch.py')),
        ('share/robot_web/www', glob('www/*')),
    ],
    install_requires=['setuptools', 'PyYAML'], zip_safe=True,
    maintainer='baobao maintainers', maintainer_email='maintainer@example.com',
    description='Baobao Gazebo mapping and navigation web console.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['robot_web_server = robot_web.server:main']},
)
